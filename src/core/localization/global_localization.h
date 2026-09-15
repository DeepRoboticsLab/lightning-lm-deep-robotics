#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/keyframe.h"

namespace lightning::loc {

// Place recognition proposes an initial pose; ordinary full-3D NDT tracking
// remains responsible for accepting the current scan after initialization.
class GlobalLocalization {
 public:
    struct Result {
        bool valid = false;
        SE3 pose;
        double confidence = 0;
    };

    GlobalLocalization();
    ~GlobalLocalization();
    static bool SaveIndex(const std::string& directory,
                          const std::vector<Keyframe::Ptr>& keyframes, bool optimized);
    bool Init(const std::string& config, const std::string& directory);
    // Single worker only. The caller owns scheduling and temporal confirmation.
    Result Search(const CloudPtr& scan);

 private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lightning::loc
