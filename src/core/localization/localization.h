#pragma once

#include <fstream>
#include <functional>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "std_msgs/msg/int32.hpp"

#include "common/imu.h"
#include "common/keyframe.h"
#include "core/localization/localization_result.h"

/// 预声明
namespace lightning {
class LaserMapping;
namespace ui {
class PangolinWindow;
}

namespace loc {

class LidarLoc;
class PGO;

/**
 * 实时定位接口实现
 */
class Localization {
   public:
    struct Options {
        Options() {}

        std::string trajectory_path_;
        bool online_mode_ = false;  // 在线模式还是离线模式
        bool with_ui_ = false;      // 是否带ui

        /// 参数
        SE3 T_body_lidar_;

        bool loc_on_kf_ = false;
        double max_frequency_ = 5.0;
    };

    Localization(Options options = Options());
    ~Localization() { Finish(); }

    /**
     * 初始化，读配置参数
     * @param yaml_path
     * @param global_map_path
     * @param init_reloc_pose
     */
    bool Init(const std::string& yaml_path, const std::string& global_map_path);

    /// 处理lidar消息
    void ProcessLidarMsg(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg);
    void ProcessLivoxLidarMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr laser_msg);

    /// 处理IMU消息
    void ProcessIMUMsg(IMUPtr imu);

    // void ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) override;

    /// 由外部设置pose，适用于手动重定位
    void SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    /// TODO: 其他初始化逻辑

    /// TODO: 处理odom消息

    /// 结束，保存临时地图
    void Finish();

    /// 异步处理函数
    void LidarLocProcCloud(CloudPtr);

    using TFCallback = std::function<void(const geometry_msgs::msg::TransformStamped& odom)>;
    using LocStateCallback = std::function<void(const std_msgs::msg::Int32& state)>;
    using PointcloudBodyCallback = std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>;
    using PointcloudWorldCallback = std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>;

    void SetTFCallback(TFCallback&& callback);

    // void SetPathCallback(std::function<void(const nav_msgs::msg::Path& path)>&& callback);
    // void SetPointcloudWorldCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetPointcloudBodyCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetLocStateCallback(std::function<void(const std_msgs::msg::Int32& state)>&& callback);
    // void SetHealthDiagNormalCallback(interface::health_diag_normal_callback&& callback);

   private:
    /// 模块  ========================================================================================================
    void ProcessBufferedLidar(bool quiet_sync = false);
    size_t lidar_messages_ = 0, imu_messages_ = 0;
    std::ofstream trajectory_;
    size_t match_count_ = 0, valid_match_count_ = 0;
    bool finished_ = false;
    std::mutex global_mutex_;  // 防止处理过程中被重复init
    Options options_;

    /// 前端
    std::shared_ptr<LaserMapping> lio_ = nullptr;
    Keyframe::Ptr lio_kf_ = nullptr;

    // ui
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;

    // pose graph
    std::shared_ptr<PGO> pgo_ = nullptr;

    // lidar localization
    std::shared_ptr<LidarLoc> lidar_loc_;

    /// TODO async 处理

    /// 结果数据 =====================================================================================================
    LocalizationResult loc_result_;

    /// 框架相关
    TFCallback tf_callback_;
    LocStateCallback loc_state_callback_;
    PointcloudBodyCallback pointcloud_body_callback_;
    PointcloudWorldCallback pointcloud_world_callback_;

    /// 输入检查
    double last_imu_time_ = 0;
    double last_odom_time_ = 0;
    double last_cloud_time_ = 0;
    double last_match_time_ = -1;
};
}  // namespace loc

}  // namespace lightning
