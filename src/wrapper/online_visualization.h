#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <mutex>

#include "common/point_def.h"

namespace lightning {

// Optional online display output. Call from the ordered sensor worker only.
// No map/keyframe clouds are retained or published here.
class OnlineVisualization {
   public:
    explicit OnlineVisualization(const rclcpp::Node::SharedPtr& node);
    void Publish(double stamp, const SE3& map_T_lidar, const CloudPtr& scan);

   private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    std::mutex path_mutex_;
    nav_msgs::msg::Path path_;
    double last_stamp_ = -1;
    rclcpp::TimerBase::SharedPtr path_timer_;
};

}  // namespace lightning
