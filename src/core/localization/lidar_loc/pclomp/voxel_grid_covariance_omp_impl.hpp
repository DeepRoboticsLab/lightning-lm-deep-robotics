/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_VOXEL_GRID_COVARIANCE_IMPL_OMP_H_
#define PCL_VOXEL_GRID_COVARIANCE_IMPL_OMP_H_

#include <pcl/common/common.h>
#include <pcl/filters/boost.h>
#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <glog/logging.h>

#include "voxel_grid_covariance_omp.h"

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint(const Eigen::MatrixXi& relative_coordinates,
                                                                const PointT& reference_point,
                                                                std::vector<LeafConstPtr>& neighbors) const {
    neighbors.clear();

    // Find displacement coordinates
    auto key = Pt2Key(Eigen::Vector3f(reference_point.x, reference_point.y, reference_point.z));
    neighbors.reserve(relative_coordinates.cols());

    // Check each neighbor to see if it is occupied and contains sufficient points
    // Slower than radius search because needs to check 26 indices
    for (int ni = 0; ni < relative_coordinates.cols(); ni++) {
        auto displacement = relative_coordinates.col(ni);
        // Checking if the specified cell is in the grid
        auto leaf_iter = leaves_.find(key + displacement);
        if (leaf_iter != leaves_.end() && leaf_iter->second.nr_points >= min_points_per_voxel_) {
            LeafConstPtr leaf = &(leaf_iter->second);
            neighbors.push_back(leaf);
        }
    }

    return (static_cast<int>(neighbors.size()));
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint(const PointT& reference_point,
                                                                std::vector<LeafConstPtr>& neighbors) const {
    neighbors.clear();

    // Find displacement coordinates
    Eigen::MatrixXi relative_coordinates = pcl::getAllNeighborCellIndices();
    return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint7(const PointT& reference_point,
                                                                 std::vector<LeafConstPtr>& neighbors) const {
    neighbors.clear();

    Eigen::MatrixXi relative_coordinates(3, 7);
    relative_coordinates.setZero();
    relative_coordinates(0, 1) = 1;
    relative_coordinates(0, 2) = -1;
    relative_coordinates(1, 3) = 1;
    relative_coordinates(1, 4) = -1;
    relative_coordinates(2, 5) = 1;
    relative_coordinates(2, 6) = -1;

    return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint1(const PointT& reference_point,
                                                                 std::vector<LeafConstPtr>& neighbors) const {
    neighbors.clear();
    return getNeighborhoodAtPoint(Eigen::MatrixXi::Zero(3, 1), reference_point, neighbors);
}

template <typename PointT>
void pclomp::VoxelGridCovariance<PointT>::AddTarget(pclomp::VoxelGridCovariance<PointT>::PointCloudPtr target) {
    for (size_t cp = 0; cp < target->points.size(); ++cp) {
        auto key = Pt2Key(target->points[cp].getVector3fMap());

        auto iter = leaves_.find(key);
        if (iter == leaves_.end()) {
            Leaf l;
            l.points_.emplace_back(target->points[cp].getVector3fMap());
            l.nr_points = 1;
            leaves_.insert({key, l});
        } else {
            auto& leaf = iter->second;
            leaf.points_.emplace_back(target->points[cp].getVector3fMap());
            leaf.nr_points++;
        }
    }
}

template <typename PointT>
void pclomp::VoxelGridCovariance<PointT>::ComputeTargetGrids() {
    std::for_each(leaves_.begin(), leaves_.end(), [this](auto& it) {
        Leaf& leaf = it.second;

        for (auto& pt : leaf.points_) {
            Eigen::Vector3d pt3d(pt[0], pt[1], pt[2]);
            // Accumulate point sum for centroid calculation
            leaf.mean_ += pt3d;
            // Accumulate x*xT for single pass covariance calculation
            leaf.cov_ += pt3d * pt3d.transpose();

            // Do we need to process all the fields?
            leaf.centroid += pt;
        }

        // Eigen values and vectors calculated to prevent near singluar matrices
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
        Eigen::Matrix3d eigen_val;
        Eigen::Vector3d pt_sum;
        double min_covar_eigvalue;

        leaf.centroid /= static_cast<float>(leaf.nr_points);
        // Point sum used for single pass covariance calculation
        pt_sum = leaf.mean_;
        // Normalize mean
        leaf.mean_ /= leaf.nr_points;

        if (leaf.nr_points < min_points_per_voxel_) {
            return;
        }

        // Single pass covariance calculation
        leaf.cov_ =
            (leaf.cov_ - 2 * (pt_sum * leaf.mean_.transpose())) / leaf.nr_points + leaf.mean_ * leaf.mean_.transpose();
        leaf.cov_ *= (leaf.nr_points - 1.0) / leaf.nr_points;

        // Normalize Eigen Val such that max no more than 100x min.
        eigensolver.compute(leaf.cov_);
        eigen_val = eigensolver.eigenvalues().asDiagonal();
        auto evecs = eigensolver.eigenvectors();

        if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0) {
            leaf.nr_points = -1;
            return;
        }

        // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]

        min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val(2, 2);
        if (eigen_val(0, 0) < min_covar_eigvalue) {
            eigen_val(0, 0) = min_covar_eigvalue;

            if (eigen_val(1, 1) < min_covar_eigvalue) {
                eigen_val(1, 1) = min_covar_eigvalue;
            }

            leaf.cov_ = evecs * eigen_val * evecs.inverse();
        }

        leaf.icov_ = leaf.cov_.inverse();
        if (leaf.icov_.maxCoeff() == std::numeric_limits<float>::infinity() ||
            leaf.icov_.minCoeff() == -std::numeric_limits<float>::infinity()) {
            leaf.nr_points = -1;
        }
    });
}

#define PCL_INSTANTIATE_VoxelGridCovariance(T) template class PCL_EXPORTS pcl::VoxelGridCovariance<T>;
#endif  // PCL_VOXEL_GRID_COVARIANCE_IMPL_H_
