#include "core/lightning_math.hpp"

#include <Eigen/SVD>
#include <pcl/filters/voxel_grid.h>

namespace lightning::math {

Eigen::Matrix<double, 2, 3> PseudoInverse(const Eigen::Matrix<double, 3, 2>& X) {
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Vec2d sv = svd.singularValues();
    Eigen::Matrix<double, 3, 2> U = svd.matrixU().block<3, 2>(0, 0);
    Eigen::Matrix<double, 2, 2> V = svd.matrixV();
    Eigen::Matrix<double, 2, 3> U_adjoint = U.adjoint();
    double tolerance = std::numeric_limits<double>::epsilon() * 3 * std::abs(sv(0, 0));
    sv(0, 0) = std::abs(sv(0, 0)) > tolerance ? 1.0 / sv(0, 0) : 0;
    sv(1, 0) = std::abs(sv(1, 0)) > tolerance ? 1.0 / sv(1, 0) : 0;

    return V * sv.asDiagonal() * U_adjoint;
}

CloudPtr VoxelGrid(CloudPtr cloud, float voxel_size) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);

    return output;
}

}  // namespace lightning::math
