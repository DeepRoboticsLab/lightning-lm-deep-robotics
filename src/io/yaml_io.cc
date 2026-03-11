//
// Created by gaoxiang on 2020/5/13.
//

#include "io/yaml_io.h"

#include <glog/logging.h>
#include <fstream>

namespace lightning {
YAML_IO::YAML_IO(const std::string &path) {
    path_ = path;
    yaml_node_ = YAML::LoadFile(path_);
    if (yaml_node_.IsNull()) {
        LOG(ERROR) << "Failed to open yaml: " << path_;
    }

    is_opened_ = true;
}

bool YAML_IO::Save(const std::string &path) {
    if (path.empty()) {
        std::ofstream fout(path_);
        fout << yaml_node_;
    } else {
        std::ofstream fout(path);
        fout << yaml_node_;
    }
    return true;
}

}  // namespace lightning
