#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <future>

#include "core/localization/lidar_loc/lidar_loc.h"
#include "core/localization/localization.h"
#include "core/localization/global_localization.h"
#include "core/lio/laser_mapping.h"
#include "core/localization/pose_graph/pgo.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"

namespace lightning::loc {

struct Localization::GlobalState {
    std::unique_ptr<GlobalLocalization> search;
    std::future<GlobalLocalization::Result> pending;
    SE3 query_lio, map_from_lio;
    double last_query_stamp = -1;
    int confirmations = 0;
    bool manual = false, initialized = false;
};

// ！ 构造函数
Localization::Localization(Options options) { options_ = options; }
Localization::~Localization() { Finish(); }

// ！初始化函数
bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (lidar_loc_ != nullptr) {
        // 若已经启动，则变为初始化
        Finish();
    }

    YAML_IO yaml(yaml_path);
    options_.with_ui_ = yaml.GetValue<bool>("system", "with_ui");

    /// lidar odom前端
    LaserMapping::Options opt_lio;
    opt_lio.is_in_slam_mode_ = false;

    lio_ = std::make_shared<LaserMapping>(opt_lio);
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio";
        return false;
    }

    /// 激光定位
    LidarLoc::Options lidar_loc_options;
    lidar_loc_options.update_dynamic_cloud_ = yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
    lidar_loc_options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
    lidar_loc_options.map_option_.enable_dynamic_polygon_ = false;
    lidar_loc_options.map_option_.map_path_ = global_map_path;
    lidar_loc_options.allow_default_initialization_ = !options_.global_init_;
    lidar_loc_ = std::make_shared<LidarLoc>(lidar_loc_options);

    if (options_.with_ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->SetCurrentScanSize(10);
        ui_->Init();

        lidar_loc_->SetUI(ui_);

        // lio_->SetUI(ui_);
    }

    if (!lidar_loc_->Init(yaml_path)) return false;
    if (options_.global_init_) {
        global_.reset(new GlobalState);
        global_->search.reset(new GlobalLocalization);
        if (!global_->search->Init(yaml_path, global_map_path)) return false;
    }
    if (!options_.trajectory_path_.empty()) {
        trajectory_.open(options_.trajectory_path_);
        if (!trajectory_) { LOG(ERROR) << "Cannot open trajectory file"; return false; }
        trajectory_ << std::setprecision(17);
    }

    /// pose graph
    pgo_ = std::make_shared<PGO>();
    pgo_->SetDebug(false);

    ///  各模块的异步调用
    options_.loc_on_kf_ = yaml.GetValue<bool>("lidar_loc", "loc_on_kf");
    options_.max_frequency_ = YAML::LoadFile(yaml_path)["lidar_loc"]["max_frequency"].as<double>(5.0);
    if (!std::isfinite(options_.max_frequency_) || options_.max_frequency_ <= 0) {
        LOG(ERROR) << "lidar_loc.max_frequency must be positive";
        return false;
    }

    /// TODO: 发布
    pgo_->SetHighFrequencyGlobalOutputHandleFunction([this](const LocalizationResult& res) {
        // if (loc_result_.timestamp_ > 0) {
        //             double loc_fps = 1.0 / (res.timestamp_ - loc_result_.timestamp_);
        //             // LOG_EVERY_N(INFO, 10) << "loc fps: " << loc_fps;
        //         }

        loc_result_ = res;

        if (tf_callback_ && loc_result_.valid_) {
            tf_callback_(loc_result_.ToGeoMsg());
        }

        if (ui_) {
            ui_->UpdateNavState(loc_result_.ToNavState());
            ui_->UpdateRecentPose(loc_result_.pose_);
        }
    });

    return true;
}

void Localization::ProcessLidarMsg(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    ++lidar_messages_;
    // Use exactly the same preprocessing and time buffering as mapping.
    lio_->ProcessPointCloud2(cloud);
    ProcessBufferedLidar();
}

void Localization::ProcessLivoxLidarMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    ++lidar_messages_;
    lio_->ProcessPointCloud2(cloud);
    ProcessBufferedLidar();
}

void Localization::ProcessBufferedLidar(bool quiet_sync) {
    if (!lio_->Run(quiet_sync)) {
        return;
    }

    auto lo_state = lio_->GetState();

    lidar_loc_->ProcessLO(lo_state);
    pgo_->ProcessLidarOdom(lo_state);

    // Keep odometry at sensor rate; bound only the more expensive map match.
    if (last_match_time_ >= 0 && lo_state.timestamp_ >= last_match_time_ &&
        lo_state.timestamp_ - last_match_time_ < 1.0 / options_.max_frequency_ - 1e-3) return;

    // LOG(INFO) << "LO pose: " << std::setprecision(12) << lo_state.timestamp_ << " "
    //           << lo_state.GetPose().translation().transpose();

    /// 获得lio的关键帧
    if (options_.loc_on_kf_) {
        auto kf = lio_->GetKeyframe();
        if (kf == lio_kf_) {
            /// 关键帧未更新，那就只更新IMU状态

            // auto dr_state = lio_->GetState();
            // lidar_loc_->ProcessDR(dr_state);
            // pgo_->ProcessDR(dr_state);
            return;
        }

        lio_kf_ = kf;

        auto scan = lio_->GetScanUndist();

        LidarLocProcCloud(scan);
    } else {
        auto scan = lio_->GetScanUndist();

        LidarLocProcCloud(scan);
    }
}

void Localization::LidarLocProcCloud(CloudPtr scan_undist) {
    last_match_time_ = lio_->GetState().timestamp_;
    if (!TryGlobalInitialization(scan_undist)) {
        ++match_count_;
        LOG(INFO) << "Global initialization waiting for an unambiguous, repeatable match";
        return;
    }
    lidar_loc_->ProcessCloud(scan_undist, lio_->GetState().timestamp_);

    auto res = lidar_loc_->GetLocalizationResult();
    if (global_ && !global_->manual && !global_->initialized) {
        global_->initialized = res.lidar_loc_valid_;
        if (!global_->initialized) global_->confirmations = 0;
    }
    LOG(INFO) << "Localization match: valid=" << res.lidar_loc_valid_ << ", confidence=" << res.confidence_;
    ++match_count_;
    if (res.lidar_loc_valid_) {
        ++valid_match_count_;
        // NDT returns map_T_lidar for this deskewed scan, at its scan-end time.
        if (visualization_callback_) visualization_callback_(res.timestamp_, res.pose_, scan_undist);
        if (trajectory_) {
            const auto t = res.pose_.translation();
            const auto q = res.pose_.unit_quaternion();
            trajectory_ << res.timestamp_ << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
                        << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        }
    }
    pgo_->ProcessLidarLoc(res);

    if (ui_) {
        // Twi with Til, here pose means Twl, thus Til=I
        ui_->UpdateScan(scan_undist, res.pose_);
    }

    if (loc_state_callback_) {
        auto loc_state = std::make_shared<std_msgs::msg::Int32>();
        loc_state->data = static_cast<int>(res.status_);
        LOG(INFO) << "loc_state: " << loc_state->data;
        loc_state_callback_(*loc_state);
    }
}

bool Localization::TryGlobalInitialization(const CloudPtr& scan) {
    if (!global_) return true;
    if (global_->manual || global_->initialized) {
        // Recognition is only needed at startup. Release its extra map/index
        // after success or a manual override, once the owned worker has finished.
        if (global_->search && (!global_->pending.valid() ||
            global_->pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready)) {
            global_->pending = std::future<GlobalLocalization::Result>();
            global_->search.reset();
        }
        return true;
    }
    auto& state = *global_;
    const auto lo = lio_->GetState();
    // Both the query pose and propagation use LiDAR coordinates at scan end.
    const SE3 local_lidar = lo.GetPose() * SE3(lo.offset_R_lidar_, lo.offset_t_lidar_);
    if (!state.pending.valid() &&
        (state.last_query_stamp < 0 || lo.timestamp_ - state.last_query_stamp >= .5)) {
        state.last_query_stamp = lo.timestamp_;
        state.query_lio = local_lidar;
        // A single owned scan and worker bound memory. Online sensor processing
        // continues while place retrieval/NDT run; offline replay stays deterministic.
        CloudPtr owned(new PointCloudType(*scan));
        state.pending = std::async(options_.online_mode_ ? std::launch::async : std::launch::deferred,
                                   [&state, owned]() { return state.search->Search(owned); });
    }
    if (state.pending.valid() && (!options_.online_mode_ ||
        state.pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready)) {
        const auto candidate = state.pending.get();
        if (!candidate.valid) {
            state.confirmations = 0;
        } else {
            const SE3 predicted = state.map_from_lio * state.query_lio;
            const SE3 change = predicted.inverse() * candidate.pose;
            if (state.confirmations && (change.translation().norm() >= 1.0 ||
                change.so3().log().norm() >= 10 * M_PI / 180)) {
                LOG(INFO) << "Global candidates disagree with odometry: translation="
                          << change.translation().norm() << " m, rotation="
                          << change.so3().log().norm() * 180 / M_PI
                          << " deg. Initialize the IMU while stationary.";
            }
            if (state.confirmations && change.translation().norm() < 1.0 &&
                change.so3().log().norm() < 10 * M_PI / 180) ++state.confirmations;
            else state.confirmations = 1;
            state.map_from_lio = candidate.pose * state.query_lio.inverse();
            LOG(INFO) << "Global initialization confirmation " << state.confirmations << "/3";
        }
    }
    if (state.confirmations < 3) return false;
    // Revalidate on the latest scan. Never publish the older worker result as
    // though it belonged to a newer scan, and never use the map-start fallback.
    lidar_loc_->SetInitialPose(state.map_from_lio * local_lidar);
    return true;
}

void Localization::ProcessIMUMsg(IMUPtr imu) {
    UL lock(global_mutex_);

    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    /// 里程计处理IMU
    ++imu_messages_;
    lio_->ProcessIMU(imu);
    if (options_.online_mode_) ProcessBufferedLidar(true);

    /// 这里需要 IMU predict，否则没法process DR了
    auto dr_state = lio_->GetIMUState();

    if (!dr_state.pose_is_ok_) {
        return;
    }

    // /// 停车判定
    // constexpr auto kThVbrbStill = 0.05;  // 0.08;
    // constexpr auto kThOmegaStill = 0.05;

    // if (dr_state.GetVel().norm() < kThVbrbStill && imu->angular_velocity.norm() < kThOmegaStill) {
    //     dr_state.is_parking_ = true;
    //     dr_state.SetVel(Vec3d::Zero());
    // }

    /// 如果没有odm, 用lio替代DR

    // LOG(INFO) << "dr state: " << std::setprecision(12) << dr_state.timestamp_ << " "
    //           << dr_state.GetPose().translation().transpose()
    //           << ", q=" << dr_state.GetPose().unit_quaternion().coeffs().transpose();

    lidar_loc_->ProcessDR(dr_state);
    pgo_->ProcessDR(dr_state);
}

// void Localization::ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
//     UL lock(global_mutex_);
//
//     if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
//         return;
//     }
//     double this_odom_time = ToSec(odom_msg->header.stamp);
//     if (last_odom_time_ > 0 && this_odom_time < last_odom_time_) {
//         LOG(WARNING) << "Odom Time Abnormal:" << this_odom_time << ", last: " << last_odom_time_;
//     }
//     last_odom_time_ = this_odom_time;
//
//     lio_->ProcessOdometry(odom_msg);
//
//     if (!lio_->GetbOdomHF()) {
//         return;
//     }
//
//     auto dr_state = lio_->GetStateHF(mapping::FasterLioMapping::kHFStateOdomFiltered);
//
//     constexpr auto kThVbrbStill = 0.03;  // 0.08;
//     constexpr auto kThOmegaStill = 0.03;
//     if (dr_state.Getvwi().norm() < kThVbrbStill && dr_state.Getwii().norm() < kThOmegaStill) {
//         dr_state.is_parking_ = true;
//         dr_state.Setvwi(Vec3d::Zero());
//         dr_state.Setwii(Vec3d::Zero());
//     }
//
//     lidar_loc_->ProcessDR(dr_state);
//     pgo_->ProcessDR(dr_state);
// }

void Localization::Finish() {
    if (finished_) return;
    finished_ = true;
    if (global_ && global_->pending.valid()) global_->pending.wait();
    if (lidar_loc_) lidar_loc_->Finish();
    if (ui_) ui_->Quit();
    trajectory_.close();
    LOG(INFO) << "Localization complete: matches=" << match_count_ << ", valid=" << valid_match_count_
              << ", lidar=" << lidar_messages_ << ", imu=" << imu_messages_;
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    if (!q.coeffs().allFinite() || !t.allFinite() || q.norm() < 1e-6) {
        LOG(ERROR) << "Initial pose must have finite coordinates and a nonzero quaternion";
        return;
    }
    /// 设置外部重定位的pose
    if (lidar_loc_) {
        if (global_) global_->manual = true;
        lidar_loc_->SetInitialPose(SE3(q.normalized(), t));
        last_match_time_ = -1;
    }
}

void Localization::SetTFCallback(Localization::TFCallback&& callback) { tf_callback_ = callback; }

}  // namespace lightning::loc
