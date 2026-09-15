#include "core/localization/global_localization.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numeric>

#include <pcl/filters/voxel_grid.h>

#include "core/localization/lidar_loc/lidar_loc.h"

namespace lightning::loc {
namespace {
constexpr int kRings = 20, kSectors = 60, kSize = kRings * kSectors;
constexpr double kRange = 80.0;
constexpr uint32_t kVersion = 1, kEndian = 0x01020304, kMaxPlaces = 20000;
constexpr char kMagic[8] = {'L', 'L', 'M', 'P', 'L', 'A', 'C', 'E'};
using Descriptor = std::array<float, kSize>;
struct Place {
    SE3 pose;
    Descriptor descriptor{};
    std::array<bool, kSectors> populated{};
};

template <class T> void Write(std::ostream& out, const T& value) {
    out.write(reinterpret_cast<const char*>(&value), sizeof(value));
}
template <class T> void Read(std::istream& in, T& value) {
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
}

// Detect an accidentally mixed/stale index without adding a crypto dependency.
uint64_t CloudFingerprint(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Cannot read map cloud: " + path);
    uint64_t hash = UINT64_C(14695981039346656037);
    std::array<char, 65536> bytes;
    while (in) {
        in.read(bytes.data(), bytes.size());
        for (std::streamsize i = 0; i < in.gcount(); ++i) {
            hash ^= static_cast<unsigned char>(bytes[i]);
            hash *= UINT64_C(1099511628211);
        }
    }
    if (!in.eof()) throw std::runtime_error("Cannot finish reading map cloud: " + path);
    return hash;
}

Descriptor Describe(const CloudPtr& cloud) {
    Descriptor result{};
    for (const auto& p : *cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const double radius = std::hypot(p.x, p.y);
        if (radius <= 1 || radius >= kRange) continue;
        const int ring = std::min(kRings - 1, int(radius * kRings / kRange));
        const int sector = int((std::atan2(p.y, p.x) + M_PI) * kSectors / (2 * M_PI)) % kSectors;
        auto& cell = result[ring * kSectors + sector];
        cell = std::max(cell, p.z + 2.0f);
    }
    return result;
}

void Normalize(Place& place) {
    for (int sector = 0; sector < kSectors; ++sector) {
        double norm = 0;
        for (int ring = 0; ring < kRings; ++ring) {
            const float v = place.descriptor[ring * kSectors + sector];
            norm += double(v) * v;
        }
        place.populated[sector] = norm > 1e-10;
        if (place.populated[sector]) {
            norm = std::sqrt(norm);
            for (int ring = 0; ring < kRings; ++ring) place.descriptor[ring * kSectors + sector] /= norm;
        }
    }
}

bool SamePlace(const SE3& a, const SE3& b, double distance, double angle) {
    return (a.translation() - b.translation()).norm() < distance &&
           (a.so3().inverse() * b.so3()).log().norm() < angle;
}
}  // namespace

struct GlobalLocalization::Impl {
    std::vector<Place> places;
    std::unique_ptr<LidarLoc> matcher;
};

GlobalLocalization::GlobalLocalization() : impl_(new Impl) {}
GlobalLocalization::~GlobalLocalization() = default;

bool GlobalLocalization::SaveIndex(const std::string& directory,
                                   const std::vector<Keyframe::Ptr>& keyframes, bool optimized) {
    if (keyframes.empty() || keyframes.size() > kMaxPlaces) {
        LOG(ERROR) << "Place index requires 1 to " << kMaxPlaces << " mapping keyframes";
        return false;
    }
    const std::string path = directory + "/places.bin";
    std::ofstream out(path + ".tmp", std::ios::binary | std::ios::trunc);
    out.write(kMagic, sizeof(kMagic));
    Write(out, kVersion); Write(out, kEndian);
    const uint32_t count = keyframes.size(); Write(out, count);
    const auto fingerprint = CloudFingerprint(directory + "/global.pcd"); Write(out, fingerprint);
    for (const auto& kf : keyframes) {
        const auto pose = optimized ? kf->GetOptPose() : kf->GetLIOPose();
        const auto t = pose.translation(); const auto q = pose.unit_quaternion();
        const std::array<double, 7> values{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
        for (double v : values) Write(out, v);
        CloudPtr sampled(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel; voxel.setLeafSize(.5f, .5f, .5f);
        voxel.setInputCloud(kf->GetCloud()); voxel.filter(*sampled);
        const auto descriptor = Describe(sampled);
        for (float v : descriptor) Write(out, v);
    }
    out.close();
    if (!out) { LOG(ERROR) << "Cannot write place index: " << path; return false; }
    std::filesystem::rename(path + ".tmp", path);
    LOG(INFO) << "Saved " << count << " mapping views for global initialization";
    return true;
}

bool GlobalLocalization::Init(const std::string& config, const std::string& directory) {
    const auto path = directory + "/places.bin";
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        LOG(ERROR) << "Global initialization needs " << path
                   << "; make a new map with this version, or use a manual initial pose";
        return false;
    }
    char magic[8]{}; uint32_t version = 0, endian = 0, count = 0; uint64_t fingerprint = 0;
    in.read(magic, sizeof(magic)); Read(in, version); Read(in, endian); Read(in, count); Read(in, fingerprint);
    const uintmax_t expected_size = 28 + uintmax_t(count) * (7 * sizeof(double) + kSize * sizeof(float));
    if (!in || !std::equal(std::begin(magic), std::end(magic), std::begin(kMagic)) ||
        version != kVersion || endian != kEndian || count == 0 || count > kMaxPlaces ||
        std::filesystem::file_size(path) != expected_size ||
        fingerprint != CloudFingerprint(directory + "/global.pcd")) {
        LOG(ERROR) << "Invalid, incompatible or stale place index: " << path;
        return false;
    }
    impl_->places.clear(); impl_->places.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        std::array<double, 7> v{}; for (double& x : v) Read(in, x);
        Quatd q(v[6], v[3], v[4], v[5]); Vec3d t(v[0], v[1], v[2]);
        if (!q.coeffs().allFinite() || !t.allFinite() || std::abs(q.norm() - 1) > 1e-3) {
            LOG(ERROR) << "Invalid pose in place index"; return false;
        }
        Place place; place.pose = SE3(q.normalized(), t);
        for (float& x : place.descriptor) {
            Read(in, x);
            if (!std::isfinite(x) || x < 0) { LOG(ERROR) << "Invalid place descriptor"; return false; }
        }
        if (!in) return false;
        Normalize(place); impl_->places.push_back(std::move(place));
    }
    LidarLoc::Options options;
    options.map_option_.map_path_ = directory;
    options.update_dynamic_cloud_ = false;
    options.force_2d_ = false;
    options.recover_pose_path_.clear();
    impl_->matcher.reset(new LidarLoc(options));
    if (!impl_->matcher->Init(config)) return false;
    LOG(INFO) << "Global initialization loaded " << count << " mapping views";
    return true;
}

GlobalLocalization::Result GlobalLocalization::Search(const CloudPtr& scan) {
    Result result;
    CloudPtr sampled(new PointCloudType);
    pcl::VoxelGrid<PointType> voxel; voxel.setLeafSize(.5f, .5f, .5f);
    voxel.setInputCloud(scan); voxel.filter(*sampled);
    if (sampled->size() < 50) return result;
    Place query; query.descriptor = Describe(sampled); Normalize(query);
    if (std::count(query.populated.begin(), query.populated.end(), true) < 6) return result;
    struct Candidate { SE3 pose; double descriptor = 0, confidence = 0; };
    std::vector<Candidate> candidates;
    for (const auto& place : impl_->places) {
        double best = 0; int best_shift = 0;
        for (int shift = 0; shift < kSectors; ++shift) {
            double sum = 0; int count = 0;
            for (int s = 0; s < kSectors; ++s) {
                const int qs = (s + kSectors - shift) % kSectors;
                // Penalize missing directions. Intersection-only normalization
                // can award a near-perfect score to a tiny accidental overlap.
                if (place.populated[s] || query.populated[qs]) ++count;
                if (!place.populated[s] || !query.populated[qs]) continue;
                for (int r = 0; r < kRings; ++r)
                    sum += place.descriptor[r * kSectors + s] * query.descriptor[r * kSectors + qs];
            }
            const double score = count ? sum / count : 0;
            if (score > best) { best = score; best_shift = shift; }
        }
        const auto yaw = SO3::exp(Vec3d(0, 0, best_shift * 2 * M_PI / kSectors));
        candidates.push_back({place.pose * SE3(yaw, Vec3d::Zero()), best, 0});
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.descriptor > b.descriptor; });
    if (candidates.empty() || candidates.front().descriptor < .55) return result;
    std::vector<Candidate> selected, accepted;
    for (auto candidate : candidates) {
        if (std::any_of(selected.begin(), selected.end(), [&](const Candidate& other) {
                return SamePlace(candidate.pose, other.pose, 8, M_PI / 4);
            })) continue;
        selected.push_back(candidate);
        if (impl_->matcher->RefineInitialCandidate(sampled, candidate.pose, candidate.confidence)) {
            accepted.push_back(candidate);
        }
        if (selected.size() == 10) break;
    }
    std::stable_sort(accepted.begin(), accepted.end(),
                     [](const Candidate& a, const Candidate& b) { return a.confidence > b.confidence; });
    if (accepted.empty() || accepted.front().confidence < 2.2 || accepted.front().descriptor < .55) return result;
    const auto& best = accepted.front();
    for (size_t i = 1; i < accepted.size(); ++i) {
        if (accepted[i].confidence >= .9 * best.confidence &&
            !SamePlace(best.pose, accepted[i].pose, 3, 20 * M_PI / 180)) {
            LOG(INFO) << "Global initialization ambiguous; waiting for another view";
            return result;
        }
    }
    result.valid = true; result.pose = best.pose; result.confidence = best.confidence;
    LOG(INFO) << "Global candidate: descriptor=" << best.descriptor << ", NDT=" << best.confidence;
    return result;
}
}  // namespace lightning::loc
