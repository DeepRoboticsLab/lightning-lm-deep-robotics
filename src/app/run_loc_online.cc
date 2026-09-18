#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "io/yaml_io.h"
#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "utils/console.h"

DEFINE_string(config, "config/m20_pro.yaml", "Sensor configuration YAML (M20 Pro or Mid360)");
DEFINE_string(map_path, "", "Map directory (localization defaults to system.map_path)");
DEFINE_string(trajectory, "", "Optional output TUM trajectory file");
DEFINE_bool(global_init, false, "Find the initial location using the map's saved place index");
DEFINE_bool(rviz, false, "Publish current LiDAR pose, scan and session trajectory for RViz2 (no map)");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::unique_ptr<lightning::console::Session> terminal;
    try {
        terminal = std::make_unique<lightning::console::Session>("Localization", true);
        tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, 4);
        rclcpp::init(argc, argv);
        lightning::YAML_IO yaml(FLAGS_config);
        lightning::LocSystem::Options options;
        options.trajectory_path_ = FLAGS_trajectory;
        options.global_init_ = FLAGS_global_init;
        options.with_rviz_ = FLAGS_rviz;
        lightning::LocSystem system(options);
        if (!system.Init(FLAGS_config, FLAGS_map_path)) return 1;
        system.Spin();
        rclcpp::shutdown();
        LOG(INFO) << "done";
        return 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        return 1;
    }
}
