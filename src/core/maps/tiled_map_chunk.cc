//
// Created by xiang on 23-2-7.
//

#include "tiled_map_chunk.h"

#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <stdexcept>

namespace lightning {

void MapChunk::AddPoint(const PointType& pt) {
    if (cloud_ == nullptr) {
        cloud_.reset(new PointCloudType);
        cloud_->reserve(50000);
    }
    cloud_->points.emplace_back(pt);
    loaded_ = true;
}

void MapChunk::LoadCloud(bool optional) {
    CloudPtr cloud(new PointCloudType);
    // Dynamic layers need not exist. Required tiles and corrupt existing files
    // still fail explicitly instead of being marked successfully loaded.
    if (!(optional && !std::filesystem::exists(filename_)) &&
        pcl::io::loadPCDFile(filename_, *cloud) < 0) {
        throw std::runtime_error("Cannot load map tile: " + filename_);
    }
    cloud_ = cloud;
    loaded_ = true;
}

void MapChunk::Unload() {
    cloud_ = nullptr;
    loaded_ = false;
}

}  // namespace lightning
