#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

class PcdMapPublisher : public rclcpp::Node
{
public:
    PcdMapPublisher()
        : Node("pcd_map_publisher")
    {
        // PCD 文件路径
        //std::string pcd_path =
        //    "/home/user/lightning-lm/data/office30/global.pcd";
            
        this->declare_parameter<std::string>(
            "pcd_path",
            "/home/user/lightning-lm/data/office30/global.pcd");
        
        this->declare_parameter<double>(
            "voxel_size",
            0.30);
        
        std::string pcd_path =
            this->get_parameter("pcd_path").as_string();
        
        double voxel_size =
            this->get_parameter("voxel_size").as_double();

        // QoS：静态地图推荐使用
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliable();
        qos.transient_local();

        // 原始地图
        global_map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/global_map",
                qos);

        // RViz 降采样地图
        global_map_vis_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/global_map_vis",
                qos);


        // ==============================
        // 1. 读取原始 PCD
        // ==============================
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(
                pcd_path,
                *cloud) == -1)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to load PCD: %s",
                pcd_path.c_str());

            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Original PCD: %zu points",
            cloud->points.size());


        // ==============================
        // 2. 发布原始地图
        // ==============================
        sensor_msgs::msg::PointCloud2 global_map_msg;

        pcl::toROSMsg(
            *cloud,
            global_map_msg);

        global_map_msg.header.frame_id = "map";
        global_map_msg.header.stamp = this->now();

        global_map_pub_->publish(global_map_msg);

        // ==============================
        // 3. VoxelGrid 降采样
        // ==============================
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        
        voxel.setInputCloud(cloud);        
              
        voxel.setLeafSize(
            static_cast<float>(voxel_size),
            static_cast<float>(voxel_size),
            static_cast<float>(voxel_size));
        
        voxel.filter(*filtered_cloud);
        
        RCLCPP_INFO(
            this->get_logger(),
            "Filtered PCD: %zu points",
            filtered_cloud->points.size());

        // ==============================
        // 4. 发布 RViz 地图
        // ==============================
        sensor_msgs::msg::PointCloud2 global_map_vis_msg;

        pcl::toROSMsg(
            *filtered_cloud,
            global_map_vis_msg);

        global_map_vis_msg.header.frame_id = "map";
        global_map_vis_msg.header.stamp = this->now();

        global_map_vis_pub_->publish(global_map_vis_msg);


        RCLCPP_INFO(
            this->get_logger(),
            "Published:");
        RCLCPP_INFO(
            this->get_logger(),
            "  /global_map");
        RCLCPP_INFO(
            this->get_logger(),
            "  /global_map_vis");
    }

private:
    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        global_map_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        global_map_vis_pub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PcdMapPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}