//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "core/localization/global_localization.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"
#include "wrapper/online_visualization.h"
#include "utils/console.h"
#include "wrapper/slam_recorder.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    if (!options_.recording_directory_.empty()) {
        recorder_ = std::make_unique<SlamRecorder>(options_.recording_directory_, yaml_path);
        lio_->SetInputCallback([this](const MeasureGroup& input) { recorder_->Record(input); });
    }
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");
        if (options_.with_rviz_) rviz_ = std::make_shared<OnlineVisualization>(node_);

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000));
        auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(16));
        const auto reliability = yaml["common"]["sensor_qos"].as<std::string>("best_effort");
        if (reliability == "best_effort") { imu_qos.best_effort(); lidar_qos.best_effort(); }
        else if (reliability != "reliable") throw std::invalid_argument("sensor_qos must be reliable or best_effort");

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                console::ReceivedImu();
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                sensor_queue_.AddMessage([this, imu]() { ProcessIMU(imu); }, sizeof(IMU) + 128);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, lidar_qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                console::ReceivedLidar();
                sensor_queue_.AddMessage([this, cloud]() { ProcessLidar(cloud); }, cloud->data.size() + sizeof(*cloud) + 256);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, lidar_qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                console::ReceivedLidar();
                sensor_queue_.AddMessage([this, cloud]() { ProcessLidar(cloud); },
                                        cloud->points.size() * sizeof(cloud->points[0]) + sizeof(*cloud) + 128);
            });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](SaveMapService::Request::SharedPtr req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        sensor_queue_.SetName("sensor input");
        sensor_queue_.SetMaxSize(4096);
        sensor_queue_.SetMaxBytes(128 * 1024 * 1024);
        sensor_queue_.SetProcFunc([](const std::function<void()>& process) { process(); });
        sensor_queue_.Start();
        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    sensor_queue_.Quit();
    FinishRecording();
    LOG(INFO) << "SLAM input: lidar=" << lidar_messages_ << ", imu=" << imu_messages_;
    if (lc_) lc_->WaitUntilIdle();
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
    console::State("mapping");
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    if (request->map_id.empty() || request->map_id == "." || request->map_id == ".." ||
        request->map_id.find_first_of("/\\") != std::string::npos) {
        LOG(ERROR) << "map_id must be a directory name";
        response->response = 2;
        return;
    }
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    try {
        response->response = SaveMap(save_path) ? 0 : 3;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Map save failed: " << e.what();
        response->response = 2;
    }
}

bool SlamSystem::SaveMap(const std::string& path) {
    sensor_queue_.WaitUntilIdle();
    if (lc_) lc_->WaitUntilIdle();
    if (lio_->GetAllKeyframes().empty()) {
        LOG(ERROR) << "No keyframes to save; provide LiDAR and IMU data first";
        return false;
    }
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    console::Event("Saving map: " + save_path);

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        if (!std::filesystem::is_empty(save_path)) {
            LOG(ERROR) << "Map directory already contains files: " << save_path;
            return false;
        }
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    if (!global_map || global_map->empty()) return false;
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    if (pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map) < 0) return false;
    if (!loc::GlobalLocalization::SaveIndex(save_path, lio_->GetAllKeyframes(), options_.with_loop_closing_)) {
        LOG(WARNING) << "Map saved without a global initialization index; use a manual initial pose";
    }
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return false;
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return false;
        }
    }

    console::Event("map saved: " + save_path);
    return true;
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    ++imu_messages_;
    if (!options_.online_mode_) console::ReceivedImu();
    lio_->ProcessIMU(imu);
    if (options_.online_mode_) ProcessBufferedLidar(true);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (!running_) return;
    ++lidar_messages_;
    if (!options_.online_mode_) console::ReceivedLidar();
    lio_->ProcessPointCloud2(cloud);
    ProcessBufferedLidar();
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (!running_) return;
    ++lidar_messages_;
    if (!options_.online_mode_) console::ReceivedLidar();
    lio_->ProcessPointCloud2(cloud);
    ProcessBufferedLidar();
}

void SlamSystem::ProcessBufferedLidar(bool quiet_sync) {
    const bool updated = lio_->Run(quiet_sync);
    HandleLidarResult(updated);
}

void SlamSystem::ProcessRecordedInput(const MeasureGroup& input) {
    if (!running_ || options_.online_mode_) throw std::logic_error("Recorded inputs require offline SLAM");
    ++lidar_messages_;
    imu_messages_ += input.imu_.size();
    console::ReceivedLidar();
    for (size_t i = 0; i < input.imu_.size(); ++i) console::ReceivedImu();
    HandleLidarResult(lio_->RunSynchronized(input));
}

bool SlamSystem::FinishRecording() {
    sensor_queue_.WaitUntilIdle();
    return !recorder_ || recorder_->Finish();
}

void SlamSystem::HandleLidarResult(bool updated) {
    auto kf = lio_->GetKeyframe();
    if (updated && rviz_) {
        const auto state = lio_->GetState();
        SE3 pose = state.GetPose();
        if (kf) pose = kf->GetOptPose() * kf->GetLIOPose().inverse() * pose;
        rviz_->Publish(state.timestamp_, pose * SE3(state.offset_R_lidar_, state.offset_t_lidar_),
                       lio_->GetScanUndist());
    }
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
