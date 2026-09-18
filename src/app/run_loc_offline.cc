#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "io/yaml_io.h"
#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "wrapper/bag_io.h"
#include "utils/console.h"

DEFINE_string(config, "config/m20_pro.yaml", "Sensor configuration YAML (M20 Pro or Mid360)");
DEFINE_string(input_bag, "", "ROS 2 bag directory or SQLite .db3 file");
DEFINE_string(map_path, "", "Map directory (localization defaults to system.map_path)");
DEFINE_string(trajectory, "", "Optional output TUM trajectory file");
DEFINE_bool(global_init, false, "Find the initial location using the map's saved place index");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::unique_ptr<lightning::console::Session> terminal;
    try {
        terminal = std::make_unique<lightning::console::Session>("Localization", false);
        tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, 4);
        if (FLAGS_input_bag.empty()) { LOG(ERROR) << "Specify --input_bag"; return 1; }
        lightning::YAML_IO yaml(FLAGS_config);
        lightning::loc::Localization::Options options;
        options.online_mode_ = false;
        options.trajectory_path_ = FLAGS_trajectory;
        options.global_init_ = FLAGS_global_init;
        lightning::loc::Localization system(options);
        const auto map_path = FLAGS_map_path.empty() ? yaml.GetValue<std::string>("system", "map_path") : FLAGS_map_path;
        if (!system.Init(FLAGS_config, map_path)) return 1;
        lightning::RosbagIO bag(FLAGS_input_bag);
        bag.AddImuHandle(yaml.GetValue<std::string>("common", "imu_topic"),
            [&](lightning::IMUPtr imu) { system.ProcessIMUMsg(imu); return true; });
        if (yaml.GetValue<int>("fasterlio", "lidar_type") == 1) {
            bag.AddLivoxCloudHandle(yaml.GetValue<std::string>("common", "livox_lidar_topic"),
                [&](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) { system.ProcessLivoxLidarMsg(cloud); return true; });
        } else {
            bag.AddPointCloud2Handle(yaml.GetValue<std::string>("common", "lidar_topic"),
                [&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) { system.ProcessLidarMsg(cloud); return true; });
        }
        bag.Go();
        system.Finish();
        if (FLAGS_global_init && !system.HasAcceptedPose()) {
            LOG(ERROR) << "No unambiguous initial pose before the recording ended. "
                          "IMU initialization requires a stationary start; a short or ambiguous view may need more data.";
            return 2;
        }
        LOG(INFO) << "done";
        return 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        return 1;
    }
}
