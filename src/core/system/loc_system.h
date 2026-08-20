//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <cstddef>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "lightning/msg/nav_state.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

namespace loc {
class Localization;
struct LocalizationResult;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;        // 是否发布 TF
        bool pub_odom_ = true;      // 是否发布 Odometry
        bool pub_nav_state_ = true; // 是否发布 NavState
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    void PublishLocalizationResult(const loc::LocalizationResult& result);
    bool PublishGlobalMap(const std::string& pcd_path);

    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    rclcpp::Publisher<msg::NavState>::SharedPtr nav_state_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ = nullptr;

    nav_msgs::msg::Path path_;
    std::size_t max_path_size_ = 2000;
    std::string global_map_pcd_path_;

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_ = nullptr;
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
