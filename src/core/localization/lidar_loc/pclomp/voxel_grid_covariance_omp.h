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

#ifndef PCL_VOXEL_GRID_COVARIANCE_OMP_H_
#define PCL_VOXEL_GRID_COVARIANCE_OMP_H_

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <map>
#include <unordered_map>

#include "core/lightning_math.hpp"

namespace pclomp {
/** \brief A searchable voxel strucure containing the mean and covariance of the data.
 * \note For more information please see
 * <b>Magnusson, M. (2009). The Three-Dimensional Normal-Distributions Transform —
 * an Efﬁcient Representation for Registration, Surface Analysis, and Loop Detection.
 * PhD thesis, Orebro University. Orebro Studies in Technology 36</b>
 * \author Brian Okorn (Space and Naval Warfare Systems Center Pacific)
 */
template <typename PointT>
class VoxelGridCovariance : public pcl::VoxelGrid<PointT> {
   protected:
    using pcl::VoxelGrid<PointT>::filter_name_;
    using pcl::VoxelGrid<PointT>::getClassName;
    using pcl::VoxelGrid<PointT>::input_;
    using pcl::VoxelGrid<PointT>::indices_;
    using pcl::VoxelGrid<PointT>::filter_limit_negative_;
    using pcl::VoxelGrid<PointT>::filter_limit_min_;
    using pcl::VoxelGrid<PointT>::filter_limit_max_;
    using pcl::VoxelGrid<PointT>::filter_field_name_;

    using pcl::VoxelGrid<PointT>::downsample_all_data_;
    using pcl::VoxelGrid<PointT>::leaf_layout_;
    using pcl::VoxelGrid<PointT>::save_leaf_layout_;
    using pcl::VoxelGrid<PointT>::leaf_size_;
    using pcl::VoxelGrid<PointT>::min_b_;
    using pcl::VoxelGrid<PointT>::max_b_;
    using pcl::VoxelGrid<PointT>::inverse_leaf_size_;
    using pcl::VoxelGrid<PointT>::div_b_;
    using pcl::VoxelGrid<PointT>::divb_mul_;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

   public:
    typedef std::shared_ptr<pcl::VoxelGrid<PointT>> Ptr;
    typedef std::shared_ptr<const pcl::VoxelGrid<PointT>> ConstPtr;

    /** \brief Simple structure to hold a centroid, covarince and the number of points in a leaf.
     * Inverse covariance, eigen vectors and engen values are precomputed. */
    struct Leaf {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** \brief Constructor.
         * Sets \ref nr_points, \ref icov_, \ref mean_ and \ref evals_ to 0 and \ref cov_ and \ref evecs_ to the
         * identity matrix
         */
        Leaf() = default;

        /** \brief Get the voxel covariance.
         * \return covariance matrix
         */
        Eigen::Matrix3d getCov() const { return (cov_); }

        /** \brief Get the inverse of the voxel covariance.
         * \return inverse covariance matrix
         */
        Eigen::Matrix3d getInverseCov() const { return (icov_); }

        /** \brief Get the voxel centroid.
         * \return centroid
         */
        Eigen::Vector3d getMean() const { return (mean_); }

        /** \brief Number of points contained by voxel */
        int nr_points = 0;

        /** \brief 3D voxel centroid */
        Eigen::Vector3d mean_ = Eigen::Vector3d::Zero();

        /** \brief Nd voxel centroid
         * \note Differs from \ref mean_ when color data is used
         */
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

        /** \brief Voxel covariance matrix */
        Eigen::Matrix3d cov_ = Eigen::Matrix3d::Identity();

        /** \brief Inverse of voxel covariance matrix */
        Eigen::Matrix3d icov_ = Eigen::Matrix3d::Zero();

        std::vector<Eigen::Vector3f> points_;
    };

    /** \brief Const pointer to VoxelGridCovariance leaf structure */
    typedef const Leaf *LeafConstPtr;

    using HashMap = std::unordered_map<Eigen::Vector3i, Leaf, lightning::math::hash_vec<3>>;

   public:
    /** \brief Constructor.
     * Sets \ref leaf_size_ to 0 and \ref searchable_ to false.
     */
    VoxelGridCovariance() : min_points_per_voxel_(6), min_covar_eigvalue_mult_(0.01), leaves_() {
        downsample_all_data_ = false;
        save_leaf_layout_ = false;
        leaf_size_.setZero();
        min_b_.setZero();
        max_b_.setZero();
        filter_name_ = "VoxelGridCovariance";
        leaves_.reserve(200000);
    }

    /** \brief Set the minimum number of points required for a cell to be used (must be 3 or greater for covariance
     * calculation). \param[in] min_points_per_voxel the minimum number of points for required for a voxel to be used
     */
    inline void setMinPointPerVoxel(int min_points_per_voxel) {
        if (min_points_per_voxel > 2) {
            min_points_per_voxel_ = min_points_per_voxel;
        } else {
            PCL_WARN("%s: Covariance calculation requires at least 3 points, setting Min Point per Voxel to 3 ",
                     this->getClassName().c_str());
            min_points_per_voxel_ = 3;
        }
    }

    /** \brief Get the minimum number of points required for a cell to be used.
     * \return the minimum number of points for required for a voxel to be used
     */
    inline int getMinPointPerVoxel() { return min_points_per_voxel_; }

    /** \brief Set the minimum allowable ratio between eigenvalues to prevent singular covariance matrices.
     * \param[in] min_covar_eigvalue_mult the minimum allowable ratio between eigenvalues
     */
    inline void setCovEigValueInflationRatio(double min_covar_eigvalue_mult) {
        min_covar_eigvalue_mult_ = min_covar_eigvalue_mult;
    }

    /** \brief Get the minimum allowable ratio between eigenvalues to prevent singular covariance matrices.
     * \return the minimum allowable ratio between eigenvalues
     */
    inline double getCovEigValueInflationRatio() { return min_covar_eigvalue_mult_; }

    inline Eigen::Vector3i Pt2Key(const Eigen::Vector3f &pt) const {
        return (pt * inverse_leaf_size_[0]).array().round().template cast<int>();
    }

    /** \brief Get the voxels surrounding point p, not including the voxel contating point p.
     * \note Only voxels containing a sufficient number of points are used (slower than radius search in practice).
     * \param[in] reference_point the point to get the leaf structure at
     * \param[out] neighbors
     * \return number of neighbors found
     */
    int getNeighborhoodAtPoint(const Eigen::MatrixXi &, const PointT &reference_point,
                               std::vector<LeafConstPtr> &neighbors) const;
    int getNeighborhoodAtPoint(const PointT &reference_point, std::vector<LeafConstPtr> &neighbors) const;
    int getNeighborhoodAtPoint7(const PointT &reference_point, std::vector<LeafConstPtr> &neighbors) const;
    int getNeighborhoodAtPoint1(const PointT &reference_point, std::vector<LeafConstPtr> &neighbors) const;

    /** \brief Get the leaf structure map
     * \return a map contataining all leaves
     */
    inline const HashMap &getLeaves() { return leaves_; }

    /// added by xiang
    /// add a target point cloud into voxel grid
    void AddTarget(PointCloudPtr target);

    /// compute the cov and inv cov of each grid, paralleled
    void ComputeTargetGrids();

   protected:
    /** \brief Minimum points contained with in a voxel to allow it to be useable. */
    int min_points_per_voxel_;

    /** \brief Minimum allowable ratio between eigenvalues to prevent singular covariance matrices. */
    double min_covar_eigvalue_mult_;

    /** \brief Voxel structure containing all leaf nodes (includes voxels with less than a sufficient number of points).
     */
    HashMap leaves_;
};

}  // namespace pclomp
#endif  // #ifndef PCL_VOXEL_GRID_COVARIANCE_H_
