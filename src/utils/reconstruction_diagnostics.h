#pragma once

#include "common/keyframe.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>

// Opt-in offline diagnostics. No solver or sensor settings are changed.
namespace lightning::diagnostics {
inline std::string Directory(const std::string& yaml_path) {
    auto yaml = YAML::LoadFile(yaml_path);
    auto node = yaml["diagnostics"]["output_directory"];
    if (!node) return {};
    auto path = node.as<std::string>();
    if (!path.empty()) std::filesystem::create_directories(path);
    return path;
}
inline void Open(std::ofstream& out, const std::string& path, const std::string& header) {
    out.open(path);
    if (!out) throw std::runtime_error("Cannot open diagnostics: " + path);
    out << std::setprecision(17) << header << '\n';
}
inline void Pose(std::ostream& out, const SE3& pose) {
    const auto& p = pose.translation();
    auto q = pose.unit_quaternion();
    out << p.x() << ',' << p.y() << ',' << p.z() << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}
inline void Keyframes(const std::string& path, const std::vector<Keyframe::Ptr>& kfs) {
    std::ofstream out;
    Open(out, path + "/keyframes.csv", "id,timestamp,raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,opt_x,opt_y,opt_z,opt_qx,opt_qy,opt_qz,opt_qw,points");
    for (const auto& kf : kfs) {
        out << kf->GetID() << ',' << kf->GetState().timestamp_ << ',';
        Pose(out, kf->GetLIOPose()); out << ','; Pose(out, kf->GetOptPose());
        out << ',' << kf->GetCloud()->size() << '\n';
    }
}
}
