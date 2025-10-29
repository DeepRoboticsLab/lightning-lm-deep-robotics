//
// Created by xiang on 25-3-12.
//

#include "io/file_io.h"
#include <filesystem>

namespace lightning {
bool PathExists(const std::string& file_path) {
    std::filesystem::path path(file_path);
    return std::filesystem::exists(path);
}

bool RemoveIfExist(const std::string& path) {
    if (PathExists(path)) {
        // LOG(INFO) << "remove " << path;
        system(("rm -f " + path).c_str());
        return true;
    }
    return false;
}

bool IsDirectory(const std::string& path) { return std::filesystem::is_directory(path); }

}  // namespace lightning