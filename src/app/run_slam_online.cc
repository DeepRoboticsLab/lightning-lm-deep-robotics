//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <csignal>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

// Global pointer to SlamSystem instance
lightning::SlamSystem* slam_instance = nullptr;

// Signal handler for SIGINT
void SignalHandler(int signum) {
    if (slam_instance) {
        LOG(INFO) << "SIGINT received, saving map and path...";
        slam_instance->SaveMap();
        slam_instance->SavePath();
    }
    rclcpp::shutdown();
    LOG(INFO) << "Shutdown complete.";
    std::exit(signum);
}

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning;

    /// Initialize ROS2
    rclcpp::init(argc, argv);

    SlamSystem::Options options;
    options.online_mode_ = true;

    SlamSystem slam(options);
    slam_instance = &slam;  // Assign global pointer

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    // Register the signal handler
    std::signal(SIGINT, SignalHandler);

    slam.StartSLAM("");
    slam.Spin();

    Timer::PrintAll();

    rclcpp::shutdown();

    LOG(INFO) << "done";

    return 0;
}