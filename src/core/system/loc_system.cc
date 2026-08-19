//
// Created by xiang on 25-9-12.
//

#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "core/localization/localization_result.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() {
    if (loc_ && map_loaded_) {
        loc_->Finish();
    }
}

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    options_.pub_tf_ = yaml.GetValue<bool>("system", "pub_tf", true);
    options_.pub_odom_ = yaml.GetValue<bool>("system", "pub_odom", true);
    options_.pub_nav_state_ = yaml.GetValue<bool>("system", "pub_nav_state", true);
    const bool enable_lidar_loc_rviz =
        yaml.GetValue<bool>("system", "enable_lidar_loc_rviz", false);
    const bool enable_path_rviz = yaml.GetValue<bool>("system", "enable_path_rviz", false);
    max_path_size_ = static_cast<std::size_t>(
        std::max(1, yaml.GetValue<int>("system", "rviz_path_max_size", 2000)));

    std::string default_global_pcd = map_path;
    if (!default_global_pcd.empty() && default_global_pcd.back() != '/') {
        default_global_pcd += '/';
    }
    default_global_pcd += "global.pcd";
    global_map_pcd_path_ =
        yaml.GetValue<std::string>("system", "rviz_global_map_pcd", default_global_pcd);

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_loc");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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

    if (yaml.GetValue<bool>("system", "enable_initialpose_rviz", true)) {
        initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                const auto &p = msg->pose.pose.position;
                const auto &o = msg->pose.pose.orientation;
                Quatd q(o.w, o.x, o.y, o.z);
                if (q.norm() < 1e-6) {
                    LOG(WARNING) << "ignore invalid /initialpose quaternion";
                    return;
                }
                q.normalize();
                SetInitPose(SE3(q, Vec3d(p.x, p.y, p.z)));
                LOG(INFO) << "received /initialpose from RViz";
            });
    }

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    if (options_.pub_odom_) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/lightning/odom", 10);
    }
    if (options_.pub_nav_state_) {
        nav_state_pub_ = node_->create_publisher<msg::NavState>("/lightning/nav_state", 10);
    }
    if (enable_path_rviz) {
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/lightning/path", 10);
    }

    if (enable_lidar_loc_rviz) {
        const std::string current_scan_topic = yaml.GetValue<std::string>(
            "system", "rviz_current_scan_topic", "/lightning/current_scan_cloud");
        const std::string global_map_topic = yaml.GetValue<std::string>(
            "system", "rviz_global_map_topic", "/lightning/global_map_cloud");

        current_scan_pub_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(current_scan_topic, 10);

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        global_map_pub_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(global_map_topic, map_qos);

        loc_->SetPointcloudWorldCallback([this](const sensor_msgs::msg::PointCloud2 &cloud) {
            if (current_scan_pub_) {
                current_scan_pub_->publish(cloud);
            }
        });
    }

    // 统一从 PGO 的全局定位结果发布 TF、Odometry、NavState 和 Path。
    loc_->SetLocalizationResultCallback(
        [this](const loc::LocalizationResult &result) { PublishLocalizationResult(result); });

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        map_loaded_ = true;
        if (global_map_pub_) {
            PublishGlobalMap(global_map_pcd_path_);
        }
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

void LocSystem::PublishLocalizationResult(const loc::LocalizationResult &result) {
    if (!result.valid_) {
        return;
    }

    const auto tf_msg = result.ToGeoMsg();
    const auto state = result.ToNavState();

    if (tf_broadcaster_) {
        tf_broadcaster_->sendTransform(tf_msg);
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = tf_msg.transform.translation.x;
    pose.position.y = tf_msg.transform.translation.y;
    pose.position.z = tf_msg.transform.translation.z;
    pose.orientation = tf_msg.transform.rotation;

    if (odom_pub_) {
        nav_msgs::msg::Odometry odom;
        odom.header = tf_msg.header;
        odom.child_frame_id = tf_msg.child_frame_id;
        odom.pose.pose = pose;
        odom.twist.twist.linear.x = state.vel_.x();
        odom.twist.twist.linear.y = state.vel_.y();
        odom.twist.twist.linear.z = state.vel_.z();
        odom_pub_->publish(odom);
    }

    if (nav_state_pub_) {
        msg::NavState nav_state;
        nav_state.header = tf_msg.header;
        nav_state.confidence = state.confidence_;
        nav_state.pose_is_ok = state.pose_is_ok_;
        nav_state.pose = pose;
        nav_state.velocity.x = state.vel_.x();
        nav_state.velocity.y = state.vel_.y();
        nav_state.velocity.z = state.vel_.z();
        nav_state_pub_->publish(nav_state);
    }

    if (path_pub_) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = tf_msg.header;
        pose_stamped.pose = pose;

        path_.header = tf_msg.header;
        path_.poses.push_back(pose_stamped);
        if (path_.poses.size() > max_path_size_) {
            path_.poses.erase(path_.poses.begin());
        }
        path_pub_->publish(path_);
    }
}

bool LocSystem::PublishGlobalMap(const std::string &pcd_path) {
    if (!global_map_pub_) {
        return false;
    }

    CloudPtr map_cloud(new PointCloudType());
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *map_cloud) != 0 || map_cloud->empty()) {
        LOG(ERROR) << "failed to load RViz global map: " << pcd_path;
        return false;
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud, map_msg);
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = node_->now();
    global_map_pub_->publish(map_msg);

    LOG(INFO) << "published RViz global map: " << pcd_path << ", points: " << map_cloud->size();
    return true;
}

}  // namespace lightning
