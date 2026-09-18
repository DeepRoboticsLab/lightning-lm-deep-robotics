#pragma once

#include <cstddef>
#include <memory>
#include <string>

namespace lightning::console {

// One session per estimator executable. No ROS/PCL dependencies in this interface.
class Session {
 public:
    Session(const std::string& mode, bool online);
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    const std::string& Directory() const { return directory_; }

 private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::string directory_;
};

void ReceivedLidar();
void ReceivedImu();
void Odometry(size_t points, size_t matched, double x, double y, double z);
void Keyframe();
void Match(bool accepted, double score, double x, double y, double z);
void State(const std::string& state);
// Durable operator messages, also included in the detailed glog file.
void Event(const std::string& message);

}  // namespace lightning::console
