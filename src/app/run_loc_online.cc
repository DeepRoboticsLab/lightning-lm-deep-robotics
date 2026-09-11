#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "io/yaml_io.h"
#include "core/system/loc_system.h"
#include "core/localization/localization.h"

DEFINE_string(config, "config/m20_pro.yaml", "Sensor configuration YAML (M20 Pro or Mid360)");
DEFINE_string(map_path, "", "Map directory (localization defaults to system.map_path)");
DEFINE_string(trajectory, "", "Optional output TUM trajectory file");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    try {
        tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, 4);
        rclcpp::init(argc, argv);
        lightning::YAML_IO yaml(FLAGS_config);
        lightning::LocSystem::Options options;
        options.trajectory_path_ = FLAGS_trajectory;
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
