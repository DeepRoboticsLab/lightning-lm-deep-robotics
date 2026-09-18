#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "io/yaml_io.h"
#include "core/system/slam.h"
#include "wrapper/bag_io.h"
#include "utils/console.h"
#include "wrapper/slam_recorder.h"

DEFINE_string(config, "config/m20_pro.yaml", "Sensor configuration YAML (M20 Pro or Mid360)");
DEFINE_string(input_bag, "", "ROS 2 bag directory or SQLite .db3 file");
DEFINE_string(map_path, "", "Map directory (localization defaults to system.map_path)");
DEFINE_bool(record_bag, false, "Record synchronized, filtered SLAM inputs beside the detailed log");
DEFINE_bool(replay_recording, false, "Replay an internal SLAM input bag without filtering/synchronizing again");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::unique_ptr<lightning::console::Session> terminal;
    try {
        terminal = std::make_unique<lightning::console::Session>("Mapping", false);
        tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, 4);
        if (FLAGS_input_bag.empty()) { LOG(ERROR) << "Specify --input_bag"; return 1; }
        lightning::YAML_IO yaml(FLAGS_config);
        lightning::SlamSystem::Options options;
        options.online_mode_ = false;
        if (FLAGS_record_bag) options.recording_directory_ = terminal->Directory();
        lightning::SlamSystem system(options);
        if (!system.Init(FLAGS_config)) return 1;
        system.StartSLAM("new_map");
        lightning::RosbagIO bag(FLAGS_input_bag);
        uint64_t sequence = 0;
        rclcpp::Serialization<lightning::msg::SlamInput> recorded_serialization;
        if (FLAGS_replay_recording) {
            bag.AddHandle(lightning::SlamRecorder::Topic, [&](const lightning::RosbagIO::MsgType& message) {
                lightning::msg::SlamInput input;
                rclcpp::SerializedMessage data(*message->serialized_data);
                recorded_serialization.deserialize_message(&data, &input);
                if (input.sequence != sequence++) throw std::runtime_error("Gap in recorded SLAM input sequence");
                system.ProcessRecordedInput(lightning::SlamRecorder::Decode(input));
                return true;
            });
        } else {
            bag.AddImuHandle(yaml.GetValue<std::string>("common", "imu_topic"),
                [&](lightning::IMUPtr imu) { system.ProcessIMU(imu); return true; });
            if (yaml.GetValue<int>("fasterlio", "lidar_type") == 1) {
                bag.AddLivoxCloudHandle(yaml.GetValue<std::string>("common", "livox_lidar_topic"),
                    [&](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) { system.ProcessLidar(cloud); return true; });
            } else {
                bag.AddPointCloud2Handle(yaml.GetValue<std::string>("common", "lidar_topic"),
                    [&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) { system.ProcessLidar(cloud); return true; });
            }
        }
        bag.Go();
        if (FLAGS_replay_recording && sequence == 0) throw std::runtime_error("No recorded SLAM inputs in bag");
        if (!system.SaveMap(FLAGS_map_path)) return 1;
        if (!system.FinishRecording()) return 2;
        LOG(INFO) << "done";
        return 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        return 1;
    }
}
