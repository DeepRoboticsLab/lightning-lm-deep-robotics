#include "wrapper/slam_recorder.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <glog/logging.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <yaml-cpp/yaml.h>
#include "utils/console.h"

namespace lightning {
namespace {
using Input = msg::SlamInput;
size_t Bytes(const Input& input) {
    return sizeof(Input) + 4 * input.xyzi.size() + 8 * (input.point_time_ms.size() + input.imu.size());
}
bool FiniteTime(double time) { return std::isfinite(time) && time >= 0 && time < 9e9; }
}

struct SlamRecorder::Impl {
    std::string directory, failure;
    size_t limit, pending_bytes = 0, peak_bytes = 0;
    uint64_t consumed = 0, written = 0, imu_written = 0, points_written = 0;
    double first_time = 0, last_time = 0;
    bool stopping = false, finished = false;
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<std::shared_ptr<Input>> queue;
    std::unique_ptr<rosbag2_cpp::Writer> writer;
    std::thread worker;

    // Caller holds mutex; reporting never waits for the disk worker.
    void Fail(const std::string& reason) {
        if (failure.empty()) {
            failure = reason;
            LOG(ERROR) << "SLAM recording INCOMPLETE: " << reason
                       << "; SLAM continues. See " << directory << "/recording.yaml";
        }
        wake.notify_one();
    }

    void Status(bool complete) {
        YAML::Node status;
        {
            std::lock_guard<std::mutex> lock(mutex);
            status["format_version"] = 1;
            status["state"] = complete ? "complete" : (failure.empty() ? "recording" : "incomplete");
            status["complete"] = complete;
            status["error"] = failure;
            status["consumed_scans"] = consumed;
            status["written_scans"] = written;
            status["unrecorded_scans"] = consumed - written;
            status["written_imu_samples"] = imu_written;
            status["written_points"] = points_written;
            status["first_scan_time"] = first_time;
            status["last_scan_time"] = last_time;
            status["buffer_limit_bytes"] = limit;
            status["peak_buffer_bytes"] = peak_bytes;
        }
        std::ofstream out(directory + "/recording.yaml.tmp");
        out.exceptions(std::ios::badbit | std::ios::failbit);
        out << status << '\n';
        out.close();
        std::filesystem::rename(directory + "/recording.yaml.tmp", directory + "/recording.yaml");
    }

    void WriteLoop() noexcept {
        try {
            rclcpp::Serialization<Input> serialization;
            auto last_status = std::chrono::steady_clock::now();
            int64_t last_stamp = 0;
            while (true) {
                std::shared_ptr<Input> input;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    wake.wait(lock, [&] { return stopping || !failure.empty() || !queue.empty(); });
                    if (queue.empty()) break;
                    input = std::move(queue.front());
                    queue.pop_front();
                }
                // All serialization and storage work runs off the sensor worker.
                auto serialized = std::make_shared<rclcpp::SerializedMessage>();
                serialization.serialize_message(input.get(), serialized.get());
                auto bag = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                bag->topic_name = SlamRecorder::Topic;
                // Maintain sequence if unusual scan spans overlap. Exact native
                // doubles remain in the message; never use these bag times in LIO.
                last_stamp = std::max(last_stamp + 1, static_cast<int64_t>(input->lidar_end_time * 1e9));
                bag->time_stamp = last_stamp;
                bag->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                    new rcutils_uint8_array_t(serialized->get_rcl_serialized_message()),
                    [serialized](rcutils_uint8_array_t* data) { delete data; });
                writer->write(bag);
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    ++written;
                    imu_written += input->imu.size() / 7;
                    points_written += input->point_time_ms.size();
                    if (written == 1) first_time = input->lidar_begin_time;
                    last_time = input->lidar_end_time;
                    pending_bytes -= Bytes(*input);
                }
                if (std::chrono::steady_clock::now() - last_status >= std::chrono::seconds(1)) {
                    Status(false);
                    last_status = std::chrono::steady_clock::now();
                }
            }
        } catch (const std::exception& error) {
            std::lock_guard<std::mutex> lock(mutex);
            Fail(error.what());
            queue.clear();
        }
        // Destruction finalizes metadata on both Foxy and Humble (no close API
        // assumption). A killed process retains the initial incomplete marker.
        writer.reset();
    }
};

SlamRecorder::SlamRecorder(const std::string& directory, const std::string& config, size_t max_bytes)
    : impl_(new Impl) {
    auto& out = *impl_;
    out.directory = directory;
    out.limit = max_bytes;
    if (max_bytes == 0) throw std::invalid_argument("recording buffer must be positive");
    std::filesystem::copy_file(config, directory + "/slam_config.yaml");
    out.Status(false);
    out.writer = std::make_unique<rosbag2_cpp::Writer>(
        std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
#if __has_include(<rosbag2_storage/storage_options.hpp>)
    rosbag2_storage::StorageOptions storage;
#else
    rosbag2_cpp::StorageOptions storage;  // Foxy
#endif
    storage.uri = directory + "/slam_input";
    storage.storage_id = "sqlite3";
    // Our bounded queue owns buffering; do not add an unbounded recorder cache.
    storage.max_cache_size = 0;
    out.writer->open(storage, {"cdr", "cdr"});
    rosbag2_storage::TopicMetadata topic;
    topic.name = Topic;
    topic.type = "lightning/msg/SlamInput";
    topic.serialization_format = "cdr";
    out.writer->create_topic(topic);
    out.worker = std::thread([&out] { out.WriteLoop(); });
    console::Event("Recording SLAM inputs: " + storage.uri);
}

SlamRecorder::~SlamRecorder() { Finish(); }

void SlamRecorder::Record(const MeasureGroup& input) noexcept {
    auto& out = *impl_;
    std::lock_guard<std::mutex> lock(out.mutex);
    ++out.consumed;
    if (!out.failure.empty() || out.stopping) return;
    try {
        if (!input.scan_ || !FiniteTime(input.lidar_begin_time_) || !FiniteTime(input.lidar_end_time_))
            throw std::runtime_error("invalid synchronized scan");
        const size_t bytes = sizeof(Input) + input.scan_->size() * 24 + input.imu_.size() * 56;
        if (bytes > out.limit || out.pending_bytes > out.limit - bytes)
            throw std::runtime_error("disk writer fell behind the bounded recording buffer");
        auto message = std::make_shared<Input>();
        message->version = 1;
        message->sequence = out.consumed - 1;
        message->lidar_begin_time = input.lidar_begin_time_;
        message->lidar_end_time = input.lidar_end_time_;
        message->xyzi.reserve(input.scan_->size() * 4);
        message->point_time_ms.reserve(input.scan_->size());
        for (const auto& p : input.scan_->points) {
            message->xyzi.insert(message->xyzi.end(), {p.x, p.y, p.z, p.intensity});
            message->point_time_ms.push_back(p.timestamp);
        }
        message->imu.reserve(input.imu_.size() * 7);
        for (const auto& imu : input.imu_) {
            message->imu.insert(message->imu.end(), {imu->timestamp,
                imu->linear_acceleration.x(), imu->linear_acceleration.y(), imu->linear_acceleration.z(),
                imu->angular_velocity.x(), imu->angular_velocity.y(), imu->angular_velocity.z()});
        }
        out.queue.push_back(std::move(message));
        out.pending_bytes += bytes;
        out.peak_bytes = std::max(out.peak_bytes, out.pending_bytes);
        out.wake.notify_one();
    } catch (const std::exception& error) { out.Fail(error.what()); }
}

bool SlamRecorder::Finish() {
    auto& out = *impl_;
    if (out.finished) return out.failure.empty();
    {
        std::lock_guard<std::mutex> lock(out.mutex);
        out.stopping = true;
    }
    out.wake.notify_one();
    if (out.worker.joinable()) out.worker.join();
    out.finished = true;
    try { out.Status(out.failure.empty() && out.written == out.consumed); }
    catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(out.mutex);
        out.Fail(std::string("cannot finalize recording status: ") + error.what());
    }
    console::Event(std::string("SLAM recording ") + (out.failure.empty() ? "complete: " : "INCOMPLETE: ") +
                   std::to_string(out.written) + "/" + std::to_string(out.consumed) + " scans, " +
                   std::to_string(out.imu_written) + " IMU samples; " + out.directory);
    return out.failure.empty();
}

MeasureGroup SlamRecorder::Decode(const msg::SlamInput& input) {
    if (input.version != 1 || input.xyzi.size() % 4 || input.imu.size() % 7 ||
        input.xyzi.size() / 4 != input.point_time_ms.size() ||
        !FiniteTime(input.lidar_begin_time) || !FiniteTime(input.lidar_end_time) ||
        input.lidar_end_time < input.lidar_begin_time)
        throw std::runtime_error("invalid recorded SLAM input layout/version/times");
    MeasureGroup group;
    group.lidar_begin_time_ = input.lidar_begin_time;
    group.lidar_end_time_ = input.lidar_end_time;
    group.scan_.reset(new PointCloudType);
    group.scan_->resize(input.point_time_ms.size());
    for (size_t i = 0; i < group.scan_->size(); ++i) {
        auto& p = group.scan_->points[i];
        p.x = input.xyzi[4*i]; p.y = input.xyzi[4*i+1]; p.z = input.xyzi[4*i+2];
        p.intensity = input.xyzi[4*i+3]; p.timestamp = input.point_time_ms[i];
    }
    for (size_t i = 0; i < input.imu.size(); i += 7) {
        auto imu = std::make_shared<IMU>();
        imu->timestamp = input.imu[i];
        imu->linear_acceleration = Vec3d(input.imu[i+1], input.imu[i+2], input.imu[i+3]);
        imu->angular_velocity = Vec3d(input.imu[i+4], input.imu[i+5], input.imu[i+6]);
        group.imu_.push_back(imu);
    }
    return group;
}
}  // namespace lightning
