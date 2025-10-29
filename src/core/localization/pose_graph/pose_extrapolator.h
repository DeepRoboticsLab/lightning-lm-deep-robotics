#ifndef TEAPROT_POSE_EXTRAPOLATOR_H_
#define TEAPROT_POSE_EXTRAPOLATOR_H_

#include <deque>

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/std_types.h"
#include "core/localization/localization_result.h"

namespace lightning::loc {

/**
 * High Frequency Pose Extrapolator
 * 高频位姿内插外推器，主要目的是外推。
 *
 * 思路：
 *
 * 要求：
 * -- 多线程安全，外部只管访问
 * -- IMU/或者DR频率【总之得高频】外推
 * -- 外推策略要保守，尤其避免过快，不要造成定位回退？
 * -- 互相校验，0.5s的误差百分比控制在x%内
 */
class PoseExtrapolator {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseExtrapolator();
    ~PoseExtrapolator();

    /// DR增量 + 内部补偿量 = 外推结果
    bool AddDRLocAndExtrapolate(const NavState& dr_loc, SE3& output_pose);
    bool AddDRLocAndExtrapolate(const NavState& dr_loc, LocalizationResult& output_loc_res);

    /// 查询同一时刻的pose，给出补偿量（只补偿平移，是相对量）
    bool AddLidarOdomLoc(const NavState& lo_loc);

    /// 重置PGO视角下的补偿量（平移+旋转，map系下的绝对量）（第一次添加PGOLoc时启动内部逻辑）
    bool AddPGOLoc(const LocalizationResult& pgo_loc);

    bool is_initialized() {
        UL lock(data_mutex_);
        return initialized;
    }

    bool GetCurrentLoc(NavState& output_nav_state);

   private:
    std::mutex data_mutex_;  // 全局数据锁

    // 原始数据
    NavState last_lo_loc_;
    LocalizationResult last_pgo_loc_;
    bool initialized = false;  // 只能被PGOLoc初始化。

    // 当前状态
    double current_time_;
    SE3 current_pose_;

    // 历史上向外发布的pose
    std::deque<NavState> output_pose_queue_;

    // 当前平移误差
    // 当前旋转误差
    bool pgo_compensate_trans_needed_ = false;
    bool pgo_compensate_rot_needed_ = false;
    Vec3d pgo_curr_trans_gap_;  // local系
    Vec3d pgo_curr_rot_gap_;    // 轴角旋转

    bool lo_compensation_needed_ = false;
    Vec3d lo_curr_trans_gap_;  // 绝对系
    Vec3d lo_curr_rot_gap_;    // 轴角旋转

    // 当前平移误差补偿速度（随DR更新，取DR平移速度的1/2；限定下限）
    // 当前旋转误差补偿速度（固定）
    double trans_velo();  // 标量速度
    double rot_velo();    // 轴角标量速度

    // 固定参数
    const double kTimeForCompensate = 0.5;  // 规定释放补偿所用的时间
    const double kMinTransVelocity = 0.5;   // 0.5米每秒，0.01米每20ms
    const double kMinRotVelocity = 0.0873;  // 5度每秒，0.1度每20ms
};

}  // namespace lightning::loc

#endif