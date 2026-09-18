//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "utils/console.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"
#include "wrapper/online_visualization.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {}
LocSystem::~LocSystem() { sensor_queue_.Quit(); if (loc_) loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path, const std::string &map_override) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    opt.trajectory_path_ = options_.trajectory_path_;
    opt.global_init_ = options_.global_init_;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = map_override.empty() ? yaml.GetValue<std::string>("system", "map_path") : map_override;
    options_.pub_tf_ = yaml.GetValue<bool>("system", "pub_tf");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_localization");
    if (options_.with_rviz_) {
        rviz_ = std::make_shared<OnlineVisualization>(node_);
        loc_->SetVisualizationCallback([this](double stamp, const SE3& pose, const CloudPtr& scan) {
            rviz_->Publish(stamp, pose, scan);
        });
    }

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000));
    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(16));
    const auto reliability = YAML::LoadFile(yaml_path)["common"]["sensor_qos"].as<std::string>("best_effort");
    if (reliability == "best_effort") { imu_qos.best_effort(); lidar_qos.best_effort(); }
    else if (reliability != "reliable") throw std::invalid_argument("sensor_qos must be reliable or best_effort");

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            console::ReceivedImu();
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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

    if (options_.pub_tf_) tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("lightning/pose", 10);
    loc_->SetTFCallback([this](const geometry_msgs::msg::TransformStamped& transform) {
        if (!rclcpp::ok()) return;
        if (tf_broadcaster_) tf_broadcaster_->sendTransform(transform);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = transform.header;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        pose_pub_->publish(pose);
    });
    initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            if (msg->header.frame_id != "map") { LOG(ERROR) << "Initial pose must use frame map"; return; }
            const auto& p = msg->pose.pose;
            Quatd q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
            if (!q.coeffs().allFinite() || q.norm() < 1e-6) return;
            q.normalize();
            const SE3 pose(q, Vec3d(p.position.x, p.position.y, p.position.z));
            sensor_queue_.AddMessage([this, pose]() { SetInitPose(pose); });
        });

    bool ret = loc_->Init(yaml_path, map_path);
    loc_started_ = ret;
    if (ret) {
        sensor_queue_.SetName("sensor input");
        sensor_queue_.SetMaxSize(4096);
        sensor_queue_.SetMaxBytes(128 * 1024 * 1024);
        sensor_queue_.SetProcFunc([](const std::function<void()>& process) { process(); });
        sensor_queue_.Start();
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

}  // namespace lightning
