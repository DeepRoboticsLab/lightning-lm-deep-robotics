//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    rclcpp::init(argc, argv);

    LocSystem::Options opt;
    // 默认开启，Init内会读取yaml覆盖这些值
    opt.pub_tf_ = true;
    opt.pub_odom_ = true;
    opt.log_pose_opt_ = true;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    /// 如果没有设置初始位姿，则从0开始
    if (opt.use_init_pose_) {
        loc.SetInitPose(opt.init_pose_);
    } else {
        loc.SetInitPose(SE3());
    }
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}