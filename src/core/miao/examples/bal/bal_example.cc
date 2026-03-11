//
// Created by xiang on 24-6-3.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <random>

#include "core/common/eigen_types.h"
#include "core/graph/base_binary_edge.h"
#include "core/graph/base_vec_vertex.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "utils/sampler.h"
#include "utils/timer.h"

#include <vector>

/**
 * \brief camera vertex which stores the parameters for a pinhole camera
 *
 * The parameters of the camera are
 * - rx,ry,rz representing the rotation axis, whereas the angle is given by
 * ||(rx,ry,rz)||
 * - tx,ty,tz the translation of the camera
 * - f the focal length of the camera
 * - k1, k2 two radial distortion parameters
 */
struct BALCam {
    SE3 Twc_;
    Vec3d f_k1_k2_ = Vec3d ::Zero();
};

class VertexCameraBAL : public miao::BaseVertex<9, BALCam> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() = default;

    void OplusImpl(const double* update) override {
        Eigen::Map<const Vec6d> upd(update);
        estimate_.Twc_ = SE3::exp(upd) * estimate_.Twc_;
        Eigen::Map<const Vec3d> upd_f(update + 6);
        estimate_.f_k1_k2_ += upd_f;
    }
};

/**
 * \brief 3D world feature
 *
 * A 3D point feature in the world
 */
class VertexPointBAL : public miao::BaseVecVertex<3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() = default;
};

/**
 * \brief edge representing the observation of a world feature by a camera
 *
 * see: http://grail.cs.washington.edu/projects/bal/
 * We use a pinhole camera model; the parameters we estimate for each camera
 * area rotation R, a translation t, a focal length f and two radial distortion
 * parameters k1 and k2. The formula for projecting a 3D point X into a camera
 * R,t,f,k1,k2 is:
 * P  =  R * X + t     (conversion from world to camera coordinates)
 * p  = -P / P.z       (perspective division)
 * p' =  f * r(p) * p  (conversion to pixel coordinates) where P.z is the third
 * (z) coordinate of P.
 *
 * In the last equation, r(p) is a function that computes a scaling factor to
 * undo the radial distortion: r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
 *
 * This gives a projection in pixels, where the origin of the image is the
 * center of the image, the positive x-axis points right, and the positive
 * y-axis points up (in addition, in the camera coordinate system, the positive
 * z-axis points backwards, so the camera is looking down the negative z-axis,
 * as in OpenGL).
 */
class EdgeObservationBAL : public miao::BaseBinaryEdge<2, Vec2d, VertexCameraBAL, VertexPointBAL> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    void ComputeError() override {
        auto v_camera = (VertexCameraBAL*)(vertices_[0]);
        auto v_point = (VertexPointBAL*)(vertices_[1]);

        BALCam camera = v_camera->Estimate();
        Vec3d point = v_point->Estimate();

        Vec3d camPt = camera.Twc_ * point;
        if (camPt.z() > 0.01) {
            error_ = Vec2d(25.0, 0.0);
        }

        Vec2d projPt(-camPt.x() / camPt.z(), -camPt.y() / camPt.z());
        double f = camera.f_k1_k2_[0];
        double k1 = camera.f_k1_k2_[1];
        double k2 = camera.f_k1_k2_[2];
        double projPtSqN = projPt.squaredNorm();
        double r = 1.0 + (k1 + k2 * projPtSqN) * projPtSqN;
        Vec2d calibPt = (f * r) * projPt;
        error_ = calibPt - measurement_;
    }

    void LinearizeOplus() override {
        auto v_camera = (VertexCameraBAL*)(vertices_[0]);
        auto v_point = (VertexPointBAL*)(vertices_[1]);

        BALCam camera = v_camera->Estimate();
        Vec3d worldPt = v_point->Estimate();

        SE3 T_W_C = camera.Twc_;

        Vec3d camPt = T_W_C * worldPt;
        if (camPt.z() > 0.01) {
            jacobian_oplus_xi_.setZero();
            jacobian_oplus_xj_.setZero();
            return;
        }

        Vec2d projPt(-camPt.x() / camPt.z(), -camPt.y() / camPt.z());
        double f = camera.f_k1_k2_[0];
        double k1 = camera.f_k1_k2_[1];
        double k2 = camera.f_k1_k2_[2];
        double projPtSqN = projPt.squaredNorm();
        double r = 1.0 + (k1 + k2 * projPtSqN) * projPtSqN;
        Vec2d calibPt = (f * r) * projPt;

        {
            const Eigen::Matrix<double, 3, 3>& R = T_W_C.so3().matrix();
            double denum = -1.0 / (camPt[2] * camPt[2]);

            // the final values of the non-calibrated case
            Eigen::Matrix<double, 2, 3> DprojPt;
            DprojPt.row(0) = (camPt[2] * R.row(0) - camPt[0] * R.row(2)) * denum;
            DprojPt.row(1) = (camPt[2] * R.row(1) - camPt[1] * R.row(2)) * denum;

            Eigen::Matrix<double, 1, 3> DpPtSqN = 2.0 * projPt.transpose() * DprojPt;
            jacobian_oplus_xj_ = (f * r) * DprojPt + projPt * DpPtSqN * (f * (k1 + k2 * 2.0 * projPtSqN));
        }

        {
            // camera pose
            Eigen::Matrix<double, 2, 6> J;

            double dz = 1 / camPt[2];
            double xdz = camPt[0] * dz;
            double ydz = camPt[1] * dz;
            double xydz2 = xdz * ydz;

            // the final values of the non-calibrated case
            Eigen::Matrix<double, 2, 6> DprojPt;
            DprojPt(0, 0) = -dz;
            DprojPt(0, 1) = 0;
            DprojPt(0, 2) = xdz * dz;
            DprojPt(0, 3) = xydz2;
            DprojPt(0, 4) = -1 - xdz * xdz;
            DprojPt(0, 5) = ydz;

            DprojPt(1, 0) = 0;
            DprojPt(1, 1) = -dz;
            DprojPt(1, 2) = ydz * dz;
            DprojPt(1, 3) = 1 + ydz * ydz;
            DprojPt(1, 4) = -xydz2;
            DprojPt(1, 5) = -xdz;

            Eigen::Matrix<double, 1, 6> DpPtSqN = 2.0 * projPt.transpose() * DprojPt;
            J = (f * r) * DprojPt + projPt * DpPtSqN * (f * (k1 + k2 * 2.0 * projPtSqN));

            jacobian_oplus_xi_.block<2, 6>(0, 0) = J;
        }

        {
            /// jacobian calib
            Eigen::Matrix<double, 2, 3> J;

            J.col(0) = r * projPt;                            // d(calibPt) / d(f)
            J.col(1) = (f * projPtSqN) * projPt;              // d(calibPt) / d(k1)
            J.col(2) = (f * projPtSqN * projPtSqN) * projPt;  // d(calibPt) / d(k2)
            jacobian_oplus_xi_.block<2, 3>(0, 6) = J;
        }
    }
};

DEFINE_int32(max_iterations, 20, "max iterations");
DEFINE_bool(use_PCG, false, "use PCG instead of the Cholesky");
DEFINE_bool(verbose, true, "verbose output");
DEFINE_string(graph_input, "./dataset/bal/problem-49-7776-pre.txt", "file which will be processed");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning::miao;

    Eigen::setNbThreads(8);
    LOG(INFO) << "use_pcg " << FLAGS_use_PCG;
    auto optimizer = SetupOptimizer<9, 3>(OptimizerConfig(
        AlgorithmType::LEVENBERG_MARQUARDT,
        FLAGS_use_PCG ? LinearSolverType::LINEAR_SOLVER_PCG : LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false));

    std::vector<std::shared_ptr<VertexPointBAL>> points;
    std::vector<std::shared_ptr<VertexCameraBAL>> cameras;

    // parse BAL dataset
    LOG(INFO) << "Loading BAL dataset " << FLAGS_graph_input;

    std::ifstream ifs(FLAGS_graph_input);
    int numCameras, numPoints, numObservations;
    ifs >> numCameras >> numPoints >> numObservations;

    LOG(INFO) << "cam: " << numCameras << ", pts: " << numPoints << ", obs: " << numObservations;

    int id = 0;
    cameras.reserve(numCameras);
    for (int i = 0; i < numCameras; ++i, ++id) {
        auto cam = std::make_shared<VertexCameraBAL>();
        cam->SetId(id);
        optimizer->AddVertex(cam);
        cameras.emplace_back(cam);
    }

    points.reserve(numPoints);
    for (int i = 0; i < numPoints; ++i, ++id) {
        auto p = std::make_shared<VertexPointBAL>();
        p->SetId(id);
        p->SetMarginalized(true);
        bool addedVertex = optimizer->AddVertex(p);
        if (!addedVertex) {
            LOG(ERROR) << "failing adding vertex";
        }
        points.emplace_back(p);
    }

    std::vector<std::shared_ptr<EdgeObservationBAL>> edges;

    // read in the observation
    for (int i = 0; i < numObservations; ++i) {
        int camIndex, pointIndex;
        double obsX, obsY;
        ifs >> camIndex >> pointIndex >> obsX >> obsY;

        assert(camIndex >= 0 && (size_t)camIndex < cameras.size() && "Index out of bounds");
        auto cam = cameras[camIndex];
        assert(pointIndex >= 0 && (size_t)pointIndex < points.size() && "Index out of bounds");
        auto point = points[pointIndex];

        auto e = std::make_shared<EdgeObservationBAL>();
        e->SetVertex(0, cam);
        e->SetVertex(1, point);
        e->SetInformation(Mat2d::Identity());
        e->SetMeasurement(Vec2d(obsX, obsY));
        bool addedEdge = optimizer->AddEdge(e);
        if (!addedEdge) {
            LOG(ERROR) << "error adding edge";
        }

        edges.emplace_back(e);
    }

    /// 罗德里格斯参数转四元数
    auto rodriguezToQuat = [](const Vec3d& rRot) {
        double angle = rRot.norm();
        if (angle < 1e-12) {
            return Quat(1, 0, 0, 0);
        }
        Vec3d v = rRot / angle;
        double c = cos(angle / 2.0), s = sin(angle / 2.0);
        return Quat(c, s * v.x(), s * v.y(), s * v.z());
    };

    // read in the camera params
    for (int i = 0; i < numCameras; ++i) {
        Vec9d data;
        for (int j = 0; j < 9; ++j) {
            ifs >> data(j);
        }

        auto cam = cameras[i];
        BALCam c;
        c.Twc_ = SE3(SO3(rodriguezToQuat(data.head<3>())), Vec3d(data[3], data[4], data[5]));
        c.f_k1_k2_ = data.tail<3>();
        cam->SetEstimate(c);
    }

    // read in the points
    for (int i = 0; i < numPoints; ++i) {
        Vec3d p;
        ifs >> p(0) >> p(1) >> p(2);
        auto point = points[i];
        point->SetEstimate(p);
    }

    optimizer->InitializeOptimization();
    optimizer->SetVerbose(FLAGS_verbose);

    Timer::Evaluate([&]() { optimizer->Optimize(FLAGS_max_iterations); }, "optimize");

    miao::Timer::DumpIntoFile("./miao_bal.txt");

    Timer::PrintAll();

    return 0;
}
