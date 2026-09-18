#pragma once

#include <memory>
#include <string>
#include "common/measure_group.h"
#include "lightning/msg/slam_input.hpp"

namespace lightning {

// One producer (the ordered SLAM worker), one disk writer. Never blocks the
// producer on storage or evicts samples. A failure ends recording explicitly.
class SlamRecorder {
 public:
    static constexpr const char* Topic = "/lightning/slam_input";
    SlamRecorder(const std::string& directory, const std::string& config,
                 size_t max_bytes = 64 * 1024 * 1024);
    ~SlamRecorder();
    void Record(const MeasureGroup& input) noexcept;
    bool Finish();
    static MeasureGroup Decode(const msg::SlamInput& input);

 private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace lightning
