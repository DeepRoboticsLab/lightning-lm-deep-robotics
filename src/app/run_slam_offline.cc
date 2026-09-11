#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "io/yaml_io.h"
#include "core/system/slam.h"
#include "wrapper/bag_io.h"

DEFINE_string(config, "config/m20_pro.yaml", "Sensor configuration YAML (M20 Pro or Mid360)");
DEFINE_string(input_bag, "", "ROS 2 bag directory or SQLite .db3 file");
DEFINE_string(map_path, "", "Map directory (localization defaults to system.map_path)");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    try {
        tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, 4);
        if (FLAGS_input_bag.empty()) { LOG(ERROR) << "Specify --input_bag"; return 1; }
        lightning::YAML_IO yaml(FLAGS_config);
        lightning::SlamSystem::Options options;
        options.online_mode_ = false;
        lightning::SlamSystem system(options);
        if (!system.Init(FLAGS_config)) return 1;
        system.StartSLAM("new_map");
        lightning::RosbagIO bag(FLAGS_input_bag);
        bag.AddImuHandle(yaml.GetValue<std::string>("common", "imu_topic"),
            [&](lightning::IMUPtr imu) { system.ProcessIMU(imu); return true; });
        if (yaml.GetValue<int>("fasterlio", "lidar_type") == 1) {
            bag.AddLivoxCloudHandle(yaml.GetValue<std::string>("common", "livox_lidar_topic"),
                [&](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) { system.ProcessLidar(cloud); return true; });
        } else {
            bag.AddPointCloud2Handle(yaml.GetValue<std::string>("common", "lidar_topic"),
                [&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) { system.ProcessLidar(cloud); return true; });
        }
        bag.Go();
        if (!system.SaveMap(FLAGS_map_path)) return 1;
        LOG(INFO) << "done";
        return 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        return 1;
    }
}
