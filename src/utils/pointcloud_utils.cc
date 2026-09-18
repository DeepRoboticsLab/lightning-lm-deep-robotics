//
// Created by xiang on 25-4-22.
//

#include "utils/pointcloud_utils.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lightning {

/// 体素滤波
CloudPtr VoxelGrid(CloudPtr cloud, float voxel_size) {
    const float inverse = 1.0f / voxel_size;
    if (!cloud || !std::isfinite(voxel_size) || voxel_size <= 0 || !std::isfinite(inverse))
        throw std::invalid_argument("VoxelGrid requires a cloud and a finite positive leaf size");
    CloudPtr output(new PointCloudType);
    output->header = cloud->header;
    output->sensor_origin_ = cloud->sensor_origin_;
    output->sensor_orientation_ = cloud->sensor_orientation_;
    output->height = 1;
    output->is_dense = true;
    if (cloud->empty()) return output;

    Eigen::Vector3f minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    Eigen::Vector3f maximum = -minimum;
    size_t finite = 0;
    for (const auto& point : *cloud) {
        const auto xyz = point.getVector3fMap();
        if (!xyz.allFinite()) continue;
        minimum = minimum.cwiseMin(xyz);
        maximum = maximum.cwiseMax(xyz);
        ++finite;
    }
    if (finite != cloud->size()) {
        LOG(WARNING) << "VoxelGrid rejected " << cloud->size() - finite << " non-finite points";
        CloudPtr valid(new PointCloudType);
        for (const auto& point : *cloud) if (point.getVector3fMap().allFinite()) valid->push_back(point);
        valid->header = cloud->header;
        valid->sensor_origin_ = cloud->sensor_origin_;
        valid->sensor_orientation_ = cloud->sensor_orientation_;
        cloud = valid;
    }
    if (!finite) return output;

    // PCL flattens its three grid coordinates into a signed 32-bit index.
    // Check absolute coordinates as well as the extent before its integer casts.
    // Use floating-point products here so the overflow guard cannot overflow.
    bool pcl_safe = true;
    long double cells = 1, pcl_extent_cells = 1;
    for (int axis = 0; axis < 3; ++axis) {
        const long double low = std::floor(minimum[axis] * inverse);
        const long double high = std::floor(maximum[axis] * inverse);
        const long double limit = std::ldexp(1.0L, 63);
        if (!std::isfinite(low) || !std::isfinite(high) || low < -limit || high >= limit)
            throw std::overflow_error("VoxelGrid coordinates exceed 64-bit indices; check point coordinates and leaf size");
        pcl_safe &= low >= std::numeric_limits<int32_t>::min() && high <= std::numeric_limits<int32_t>::max();
        cells *= high - low + 1;
        pcl_extent_cells *= std::floor((maximum[axis] - minimum[axis]) * inverse) + 1;
    }
    pcl_safe &= cells <= std::numeric_limits<int32_t>::max() &&
                pcl_extent_cells <= std::numeric_limits<int32_t>::max();
    if (!pcl_safe) {
        // Preserve the requested resolution and centroid semantics. Sparse keys
        // require memory proportional to point count, not the bounding-box volume.
        struct Entry { std::array<int64_t, 3> key; size_t index; };
        std::vector<Entry> entries;
        entries.reserve(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto xyz = (*cloud)[i].getVector3fMap();
            entries.push_back({{static_cast<int64_t>(std::floor(xyz.z() * inverse)),
                                static_cast<int64_t>(std::floor(xyz.y() * inverse)),
                                static_cast<int64_t>(std::floor(xyz.x() * inverse))}, i});
        }
        std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
            return a.key == b.key ? a.index < b.index : a.key < b.key;
        });
        for (size_t first = 0; first < entries.size();) {
            size_t end = first;
            pcl::CentroidPoint<PointType> centroid;
            while (end < entries.size() && entries[end].key == entries[first].key)
                centroid.add((*cloud)[entries[end++].index]);
            PointType point;
            point.getVector4fMap().setZero();
            point.intensity = 0;
            point.timestamp = 0;  // PCL's centroid does not aggregate this custom field.
            centroid.get(point);
            output->push_back(point);
            first = end;
        }
        LOG(INFO) << "VoxelGrid used sparse 64-bit indices: leaf=" << voxel_size
                  << ", input=" << cloud->size() << ", output=" << output->size();
        return output;
    }
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    voxel.filter(*output);
    return output;
}

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min) {
    CloudPtr output(new PointCloudType);
    for (const auto &pt : cloud->points) {
        if (pt.z > z_min) {
            output->points.emplace_back(pt);
        }
    }

    output->height = 1;
    output->is_dense = false;
    output->width = output->points.size();
    cloud->swap(*output);
}

}  // namespace lightning
