#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <vector>

// Independent saved-map viewer utility. It never loads or changes estimator state.
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<rclcpp::Node>("lightning_saved_map");
    try {
        const auto path = node->declare_parameter<std::string>("pcd_file", "");
        const auto info_path = node->declare_parameter<std::string>("info_file", "");
        const auto limit = node->declare_parameter<int64_t>("max_points", 500000);
        const auto xyz = node->declare_parameter<std::vector<double>>("display_translation", {0., 0., 0.});
        const auto xyzw = node->declare_parameter<std::vector<double>>("display_quaternion", {0., 0., 0., 1.});
        if (xyz.size() != 3 || xyzw.size() != 4 ||
            !std::all_of(xyz.begin(), xyz.end(), [](double v) { return std::isfinite(v); }) ||
            !std::all_of(xyzw.begin(), xyzw.end(), [](double v) { return std::isfinite(v); })) {
            throw std::runtime_error("Display translation/quaternion must contain 3/4 finite values.");
        }
        Eigen::Quaterniond rotation(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
        if (rotation.norm() < 1e-9) throw std::runtime_error("Invalid display quaternion.");
        rotation.normalize();
        const Eigen::Vector3d translation(xyz[0], xyz[1], xyz[2]);
        if (path.empty() || limit < 1 || limit > 500000) {
            throw std::runtime_error("Set pcd_file and max_points between 1 and 500000.");
        }
        pcl::PCLPointCloud2 stored;
        if (pcl::io::loadPCDFile(path, stored) < 0 || stored.data.empty()) {
            throw std::runtime_error("Cannot read a nonempty PCD: " + path);
        }
        for (const auto* name : {"x", "y", "z"}) {
            const auto field = std::find_if(stored.fields.begin(), stored.fields.end(),
                [name](const pcl::PCLPointField& item) { return item.name == name; });
            if (field == stored.fields.end() || field->datatype != pcl::PCLPointField::FLOAT32 ||
                field->count != 1) {
                throw std::runtime_error("The map must contain float32 x, y and z fields.");
            }
        }
        pcl::PointCloud<pcl::PointXYZ> input;
        pcl::fromPCLPointCloud2(stored, input);
        stored.data.clear();
        stored.data.shrink_to_fit();
        const auto finite = [](const pcl::PointXYZ& p) {
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        };
        std::array<double, 3> lower, upper;
        lower.fill(std::numeric_limits<double>::infinity());
        upper.fill(-std::numeric_limits<double>::infinity());
        size_t valid = 0;
        for (const auto& p : input) {
            if (!finite(p)) continue;
            ++valid;
            for (size_t axis = 0; axis < 3; ++axis) {
                lower[axis] = std::min(lower[axis], static_cast<double>(p.data[axis]));
                upper[axis] = std::max(upper[axis], static_cast<double>(p.data[axis]));
            }
        }
        if (valid == 0) throw std::runtime_error("The PCD has no finite XYZ points.");
        // Uniform display sampling bounds a single XYZ message below 8 MiB.
        // Full input bounds determine the camera; the original PCD stays untouched.
        const size_t stride = (valid + static_cast<size_t>(limit) - 1) / static_cast<size_t>(limit);
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = node->now();
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize((valid + stride - 1) / stride);
        sensor_msgs::PointCloud2Iterator<float> out(cloud, "x");
        size_t index = 0;
        for (const auto& p : input) {
            if (!finite(p) || index++ % stride != 0) continue;
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            ++out;
        }
        cloud.is_dense = true;
        auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lightning/saved_map", rclcpp::QoS(1).transient_local());
        publisher->publish(cloud);
        // A private, retained frame also makes a standalone map valid in RViz
        // with no sensor/SLAM TF publisher. Cloud coordinates remain untouched.
        tf2_msgs::msg::TFMessage frames;
        frames.transforms.resize(1);
        auto& transform = frames.transforms.front();
        transform.header.stamp = cloud.header.stamp;
        transform.header.frame_id = "lightning_map_view";
        transform.child_frame_id = "map";
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        transform.transform.rotation.x = rotation.x();
        transform.transform.rotation.y = rotation.y();
        transform.transform.rotation.z = rotation.z();
        transform.transform.rotation.w = rotation.w();
        auto frame_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>(
            "/lightning/saved_map_tf", rclcpp::QoS(1).transient_local());
        frame_publisher->publish(frames);
        RCLCPP_INFO(node->get_logger(), "Saved map: %zu input, %zu finite, %u displayed points (%zu bytes)",
                    input.size(), valid, cloud.width, cloud.data.size());
        if (!info_path.empty()) {
            std::array<double, 3> center;
            double radius_squared = 0;
            for (size_t axis = 0; axis < 3; ++axis) {
                center[axis] = (lower[axis] + upper[axis]) * 0.5;
                radius_squared += std::pow((upper[axis] - lower[axis]) * 0.5, 2);
            }
            const Eigen::Vector3d display_center = rotation * Eigen::Vector3d(center.data()) + translation;
            const auto temporary = info_path + ".tmp";
            std::ofstream info(temporary);
            info << std::setprecision(17) << "{\"center\":[" << display_center.x() << "," << display_center.y() << ","
                 << display_center.z() << "],\"radius\":" << std::sqrt(radius_squared)
                 << ",\"input_points\":" << input.size() << ",\"finite_points\":" << valid
                 << ",\"display_points\":" << cloud.width << ",\"message_bytes\":" << cloud.data.size() << "}\n";
            info.close();
            if (!info || std::rename(temporary.c_str(), info_path.c_str()) != 0) {
                throw std::runtime_error("Cannot write map-view metadata: " + info_path);
            }
        }
        input.clear();
        input.points.shrink_to_fit();
        rclcpp::spin(node);  // Retain the map for viewers that subscribe after loading.
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
}
