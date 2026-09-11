#include "wrapper/online_visualization.h"

#include <cmath>
#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lightning {

OnlineVisualization::OnlineVisualization(const rclcpp::Node::SharedPtr& node) {
    pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("lightning/current_pose", 1);
    scan_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lightning/current_scan", rclcpp::QoS(1).best_effort());
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
        "lightning/trajectory", rclcpp::QoS(1).transient_local());
    tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>("lightning/tf", 1);
    path_.header.frame_id = "map";
    // A wall timer also delivers the final samples when sensor input stops.
    // Copy under the lock; DDS serialization must not hold the sensor worker.
    path_timer_ = node->create_wall_timer(std::chrono::seconds(1), [this]() {
        nav_msgs::msg::Path snapshot;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            if (path_.poses.empty()) return;
            snapshot = path_;
        }
        path_pub_->publish(snapshot);
    });
}

void OnlineVisualization::Publish(double stamp, const SE3& map_T_lidar, const CloudPtr& scan) {
    if (!rclcpp::ok() || !std::isfinite(stamp) || stamp < 0 || !map_T_lidar.matrix().allFinite() ||
        !scan || scan->empty()) return;
    // Display at at most 5 Hz without changing acquisition or estimation rates.
    if (last_stamp_ >= 0 && stamp - last_stamp_ < 0.2 - 1e-3) return;
    last_stamp_ = stamp;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(stamp * 1e9)));
    const auto& p = map_T_lidar.translation();
    const auto q = map_T_lidar.unit_quaternion();
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    tf2_msgs::msg::TFMessage tf;
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = "lightning_lidar";
    transform.transform.translation.x = p.x();
    transform.transform.translation.y = p.y();
    transform.transform.translation.z = p.z();
    transform.transform.rotation = pose.pose.orientation;
    tf.transforms.push_back(transform);
    tf_pub_->publish(tf);

    // Keep the whole sampled session, including poses recorded before RViz opens.
    // Historical points are estimates at acquisition time, not an optimized map.
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_.header = pose.header;
        path_.poses.push_back(pose);
    }

    if (scan_pub_->get_subscription_count() == 0) return;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = pose.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(scan->size());
    sensor_msgs::PointCloud2Iterator<float> out(cloud, "x");
    size_t count = 0;
    for (const auto& point : scan->points) {
        const Vec3d xyz = map_T_lidar * point.getVector3fMap().cast<double>();
        if (!xyz.allFinite()) continue;
        out[0] = static_cast<float>(xyz.x());
        out[1] = static_cast<float>(xyz.y());
        out[2] = static_cast<float>(xyz.z());
        out[3] = std::isfinite(point.intensity) ? point.intensity : 0.0f;
        ++out;
        ++count;
    }
    modifier.resize(count);
    cloud.is_dense = true;
    scan_pub_->publish(cloud);
}

}  // namespace lightning
