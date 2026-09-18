#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "io/yaml_io.h"
#include "core/system/slam.h"
#include "utils/console.h"

DEFINE_string(config, "config/m20_pro.yaml", "Sensor configuration YAML (M20 Pro or Mid360)");
DEFINE_string(map_name, "new_map", "Name of the mapping session");
DEFINE_bool(rviz, false, "Publish current LiDAR pose, scan and session trajectory for RViz2 (no map)");
DEFINE_bool(record_bag, false, "Record synchronized, filtered SLAM inputs beside the detailed log");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::unique_ptr<lightning::console::Session> terminal;
    try {
        terminal = std::make_unique<lightning::console::Session>("Mapping", true);
        tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, 4);
        rclcpp::init(argc, argv);
        lightning::YAML_IO yaml(FLAGS_config);
        lightning::SlamSystem::Options options;
        options.online_mode_ = true;
        options.with_rviz_ = FLAGS_rviz;
        if (FLAGS_record_bag) options.recording_directory_ = terminal->Directory();
        lightning::SlamSystem system(options);
        if (!system.Init(FLAGS_config)) return 1;
        system.StartSLAM(FLAGS_map_name);
        system.Spin();
        const bool recording_ok = system.FinishRecording();
        rclcpp::shutdown();
        LOG(INFO) << "done";
        return recording_ok ? 0 : 2;
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        return 1;
    }
}
