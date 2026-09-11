#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "io/yaml_io.h"
#include "core/system/slam.h"

DEFINE_string(config, "config/m20_pro.yaml", "Sensor configuration YAML (M20 Pro or Mid360)");
DEFINE_string(map_name, "new_map", "Name of the mapping session");
DEFINE_bool(rviz, false, "Publish current LiDAR pose, scan and session trajectory for RViz2 (no map)");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    try {
        tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, 4);
        rclcpp::init(argc, argv);
        lightning::YAML_IO yaml(FLAGS_config);
        lightning::SlamSystem::Options options;
        options.online_mode_ = true;
        options.with_rviz_ = FLAGS_rviz;
        lightning::SlamSystem system(options);
        if (!system.Init(FLAGS_config)) return 1;
        system.StartSLAM(FLAGS_map_name);
        system.Spin();
        rclcpp::shutdown();
        LOG(INFO) << "done";
        return 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        return 1;
    }
}
