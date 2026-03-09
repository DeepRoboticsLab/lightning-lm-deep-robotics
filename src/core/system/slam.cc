//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_rviz_visualization_ = yaml["system"]["enable_lidar_loc_rviz"] ? yaml["system"]["enable_lidar_loc_rviz"].as<bool>() : false;
    options_.pub_tf_ = yaml["system"]["pub_tf"] ? yaml["system"]["pub_tf"].as<bool>() : false;
    options_.pub_odom_ = yaml["system"]["pub_odom"] ? yaml["system"]["pub_odom"].as<bool>() : false;
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();
    options_.log_pose_opt_ = yaml["system"]["log_pose_opt"] ? yaml["system"]["log_pose_opt"].as<bool>() : false;

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

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        rclcpp::QoS qos(10);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        if (options_.with_rviz_visualization_) {
            std::string scan_topic = yaml["system"]["rviz_current_scan_topic"] ? yaml["system"]["rviz_current_scan_topic"].as<std::string>() : "lightning/current_scan";
            std::string map_topic = yaml["system"]["rviz_global_map_topic"] ? yaml["system"]["rviz_global_map_topic"].as<std::string>() : "lightning/global_map";
            cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, 10);
            map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, 1);
            path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("lightning/path", 10);
        }

        if (options_.pub_odom_) {
            odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("lightning/odom", 10);
            nav_state_pub_ = node_->create_publisher<msg::NavState>("lightning/nav_state", 10);
        }

        if (options_.pub_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        }

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
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
            return;  // 文件打开失败
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
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    if (options_.log_pose_opt_) {
        auto state = lio_->GetState();
        auto q = state.rot_.unit_quaternion();
        LOG(INFO) << "pose: " << state.pos_.transpose() << "\tq: " << q.coeffs().transpose() << "\tvelocity: " << state.vel_.transpose();

        // printf("\rslam.cc:266] pose: [%.3f, %.3f, %.3f], q: [%.3f, %.3f, %.3f, %.3f], vel: [%.3f, %.3f, %.3f]           ", 
        //        state.pos_.x(), state.pos_.y(), state.pos_.z(),
        //        q.w(), q.x(), q.y(), q.z(),
        //        state.vel_.x(), state.vel_.y(), state.vel_.z());
        // fflush(stdout);
    }

    if (options_.with_rviz_visualization_ && cloud_pub_ != nullptr) {
        auto cloud_undistort = lio_->GetScanUndist();
        if (cloud_undistort) {
            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(*cloud_undistort, ros_cloud);
            ros_cloud.header.frame_id = "map";
            ros_cloud.header.stamp = node_->now();
            cloud_pub_->publish(ros_cloud);
        }
    }

    auto kf = lio_->GetKeyframe();
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

    if (cloud_pub_ != nullptr) {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cur_kf_->GetCloud(), ros_cloud);
        ros_cloud.header.frame_id = "map";
        ros_cloud.header.stamp = node_->now();
        cloud_pub_->publish(ros_cloud);
    }

    if (nav_state_pub_ != nullptr) {
        auto state = lio_->GetState();
        msg::NavState ns;
        ns.header.stamp = node_->now();
        ns.header.frame_id = "map";
        ns.pose.position.x = state.pos_.x();
        ns.pose.position.y = state.pos_.y();
        ns.pose.position.z = state.pos_.z();
        auto q = state.rot_.unit_quaternion();
        ns.pose.orientation.x = q.x();
        ns.pose.orientation.y = q.y();
        ns.pose.orientation.z = q.z();
        ns.pose.orientation.w = q.w();
        ns.velocity.x = state.vel_.x();
        ns.velocity.y = state.vel_.y();
        ns.velocity.z = state.vel_.z();
        ns.confidence = 1.0;
        ns.pose_is_ok = true;
        nav_state_pub_->publish(ns);

        if (tf_broadcaster_ != nullptr) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = ns.header;
            tf_msg.child_frame_id = "lidar_link";
            tf_msg.transform.translation.x = ns.pose.position.x;
            tf_msg.transform.translation.y = ns.pose.position.y;
            tf_msg.transform.translation.z = ns.pose.position.z;
            tf_msg.transform.rotation = ns.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }

        if (odom_pub_ != nullptr) {
            nav_msgs::msg::Odometry odom;
            odom.header = ns.header;
            odom.child_frame_id = "lidar_link";
            odom.pose.pose = ns.pose;
            odom.twist.twist.linear = ns.velocity;
            odom_pub_->publish(odom);
        }

        if (path_pub_ != nullptr) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = ns.header;
            ps.pose = ns.pose;
            path_.header = ns.header;
            path_.poses.push_back(ps);
            path_pub_->publish(path_);
        }
    }

    if (map_pub_ != nullptr) {
        global_map_clouds_[cur_kf_->GetID()] = cur_kf_->GetCloud();
        
        CloudPtr global_map(new PointCloudType);
        for (const auto& kf_cloud : global_map_clouds_) {
            *global_map += *kf_cloud.second;
        }

        sensor_msgs::msg::PointCloud2 ros_map;
        pcl::toROSMsg(*global_map, ros_map);
        ros_map.header.frame_id = "map";
        ros_map.header.stamp = node_->now();
        map_pub_->publish(ros_map);
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    if (options_.log_pose_opt_) {
        auto state = lio_->GetState();
        auto q = state.rot_.unit_quaternion();
        LOG(INFO) << "pose: " << state.pos_.transpose() << "\tq: " << q.coeffs().transpose() << "\tvelocity: " << state.vel_.transpose();

        // printf("\rslam.cc:374] pose: [%.3f, %.3f, %.3f], q: [%.3f, %.3f, %.3f, %.3f], vel: [%.3f, %.3f, %.3f]           ", 
        //        state.pos_.x(), state.pos_.y(), state.pos_.z(),
        //        q.w(), q.x(), q.y(), q.z(),
        //        state.vel_.x(), state.vel_.y(), state.vel_.z());
        // fflush(stdout);
    }

    if (options_.with_rviz_visualization_ && cloud_pub_ != nullptr) {
        auto cloud_undistort = lio_->GetScanUndist();
        if (cloud_undistort) {
            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(*cloud_undistort, ros_cloud);
            ros_cloud.header.frame_id = "map";
            ros_cloud.header.stamp = node_->now();
            cloud_pub_->publish(ros_cloud);
        }
    }

    auto kf = lio_->GetKeyframe();
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

    if (cloud_pub_ != nullptr) {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cur_kf_->GetCloud(), ros_cloud);
        ros_cloud.header.frame_id = "map";
        ros_cloud.header.stamp = node_->now();
        cloud_pub_->publish(ros_cloud);
    }

    if (nav_state_pub_ != nullptr) {
        auto state = lio_->GetState();
        msg::NavState ns;
        ns.header.stamp = node_->now();
        ns.header.frame_id = "map";
        ns.pose.position.x = state.pos_.x();
        ns.pose.position.y = state.pos_.y();
        ns.pose.position.z = state.pos_.z();
        auto q = state.rot_.unit_quaternion();
        ns.pose.orientation.x = q.x();
        ns.pose.orientation.y = q.y();
        ns.pose.orientation.z = q.z();
        ns.pose.orientation.w = q.w();
        ns.velocity.x = state.vel_.x();
        ns.velocity.y = state.vel_.y();
        ns.velocity.z = state.vel_.z();
        ns.confidence = 1.0;
        ns.pose_is_ok = true;
        nav_state_pub_->publish(ns);

        if (tf_broadcaster_ != nullptr) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = ns.header;
            tf_msg.child_frame_id = "lidar_link";
            tf_msg.transform.translation.x = ns.pose.position.x;
            tf_msg.transform.translation.y = ns.pose.position.y;
            tf_msg.transform.translation.z = ns.pose.position.z;
            tf_msg.transform.rotation = ns.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }

        if (odom_pub_ != nullptr) {
            nav_msgs::msg::Odometry odom;
            odom.header = ns.header;
            odom.child_frame_id = "lidar_link";
            odom.pose.pose = ns.pose;
            odom.twist.twist.linear = ns.velocity;
            odom_pub_->publish(odom);
        }

        if (path_pub_ != nullptr) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = ns.header;
            ps.pose = ns.pose;
            path_.header = ns.header;
            path_.poses.push_back(ps);
            path_pub_->publish(path_);
        }
    }

    if (map_pub_ != nullptr) {
        global_map_clouds_[cur_kf_->GetID()] = cur_kf_->GetCloud();
        
        CloudPtr global_map(new PointCloudType);
        for (const auto& kf_cloud : global_map_clouds_) {
            *global_map += *kf_cloud.second;
        }

        sensor_msgs::msg::PointCloud2 ros_map;
        pcl::toROSMsg(*global_map, ros_map);
        ros_map.header.frame_id = "map";
        ros_map.header.stamp = node_->now();
        map_pub_->publish(ros_map);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
