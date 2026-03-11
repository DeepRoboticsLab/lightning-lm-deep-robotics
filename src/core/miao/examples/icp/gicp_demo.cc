//
// Created by xiang on 24-5-20.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <cstdint>
#include <iostream>
#include <random>

#include "core/common/eigen_types.h"
#include "core/graph/base_binary_edge.h"
#include "core/graph/base_vertex.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "utils/sampler.h"
#include "utils/timer.h"

#include "core/types/vertex_se3.h"

class EdgeGICP {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

   public:
    // point positions
    Vec3d pos0_ = Vec3d::Zero(), pos1_ = Vec3d::Zero();

    // unit normals
    Vec3d normal0_, normal1_;

    // rotation matrix for normal
    Mat3d R0_, R1_;

    // initialize an object
    EdgeGICP() {
        normal0_ << 0, 0, 1;
        normal1_ << 0, 0, 1;
        // makeRot();
        R0_.setIdentity();
        R1_.setIdentity();
    }

    // set up rotation matrix for pos0
    void makeRot0() {
        Vector3 y;
        y << 0, 1, 0;
        R0_.row(2) = normal0_;
        y = y - normal0_(1) * normal0_;
        y.normalize();  // need to check if y is close to 0
        R0_.row(1) = y;
        R0_.row(0) = normal0_.cross(R0_.row(1));
    }

    // set up rotation matrix for pos1
    void makeRot1() {
        Vector3 y;
        y << 0, 1, 0;
        R1_.row(2) = normal1_;
        y = y - normal1_(1) * normal1_;
        y.normalize();  // need to check if y is close to 0
        R1_.row(1) = y;
        R1_.row(0) = normal1_.cross(R1_.row(1));
    }

    // returns a precision matrix for point-plane
    Matrix3 prec0(double e) {
        makeRot0();
        Matrix3 prec;
        prec << e, 0, 0, 0, e, 0, 0, 0, 1;
        return R0_.transpose() * prec * R0_;
    }

    // returns a precision matrix for point-plane
    Matrix3 prec1(double e) {
        makeRot1();
        Matrix3 prec;
        prec << e, 0, 0, 0, e, 0, 0, 0, 1;
        return R1_.transpose() * prec * R1_;
    }

    // return a covariance matrix for plane-plane
    Matrix3 cov0(double e) {
        makeRot0();
        Matrix3 cov;
        cov << 1, 0, 0, 0, 1, 0, 0, 0, e;
        return R0_.transpose() * cov * R0_;
    }

    // return a covariance matrix for plane-plane
    Matrix3 cov1(double e) {
        makeRot1();
        Matrix3 cov;
        cov << 1, 0, 0, 0, 1, 0, 0, 0, e;
        return R1_.transpose() * cov * R1_;
    }
};

// 3D rigid constraint
//    3 values for position wrt frame
//    3 values for normal wrt frame, not used here
// first two args are the measurement type, second two the connection classes
class Edge_V_V_GICP : public miao::BaseBinaryEdge<3, EdgeGICP, miao::VertexSE3, miao::VertexSE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Edge_V_V_GICP() : pl_pl(false) {}

    // switch to go between point-plane and plane-plane
    bool pl_pl;
    Matrix3 cov0, cov1;

    // return the error estimate as a 3-vector
    void ComputeError() override {
        // from <ViewPoint> to <Point>
        const auto vp0 = (miao::VertexSE3*)(vertices_[0]);
        const auto vp1 = (miao::VertexSE3*)(vertices_[1]);

        // get vp1 point into vp0 frame
        // could be more efficient if we computed this transform just once
        Vector3 p1;

        p1 = vp1->Estimate() * measurement_.pos1_;
        p1 = vp0->Estimate().inverse() * p1;

        // get their difference
        // this is simple Euclidean distance, for now
        error_ = p1 - measurement_.pos0_;

        if (!pl_pl) {
            return;
        }

        // re-define the information matrix
        // topLeftCorner<3,3>() is the rotation()
        const Matrix3 transform = (vp0->Estimate().inverse() * vp1->Estimate()).matrix().topLeftCorner<3, 3>();
        information_ = (cov0 + transform * cov1 * transform.transpose()).inverse();
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning::miao;

    double euc_noise = 0.01;  // noise in position, m
    //  double outlier_ratio = 0.1;

    Optimizer optimizer;
    optimizer.SetVerbose(false);

    SetupOptimizer(optimizer, OptimizerConfig(AlgorithmType::LEVENBERG_MARQUARDT));

    std::vector<Vec3d> true_points;
    for (size_t i = 0; i < 1000; ++i) {
        true_points.emplace_back((Sampler::uniformRand(0., 1.) - 0.5) * 3, Sampler::uniformRand(0., 1.) - 0.5,
                                 Sampler::uniformRand(0., 1.) + 10);
    }

    // set up two poses
    int vertex_id = 0;

    std::shared_ptr<VertexSE3> vp0, vp1;

    for (size_t i = 0; i < 2; ++i) {
        // set up rotation and translation for this node
        Vec3d t(0, 0, i);
        Quat q;
        q.setIdentity();

        SE3 cam(q, t);  // camera pose

        // set up node
        auto vc = std::make_shared<VertexSE3>();

        vc->SetEstimate(cam);
        vc->SetId(vertex_id);  // vertex id

        // set first cam pose fixed
        if (i == 0) {
            vc->SetFixed(true);
            vp0 = vc;
        } else {
            vp1 = vc;
        }

        // add to optimizer
        optimizer.AddVertex(vc);

        vertex_id++;
    }

    // set up point matches
    for (size_t i = 0; i < true_points.size(); ++i) {
        // calculate the relative 3D position of the point
        Vec3d pt0, pt1;
        pt0 = vp0->Estimate().inverse() * true_points[i];
        pt1 = vp1->Estimate().inverse() * true_points[i];

        // add in noise
        pt0 += Vec3d(Sampler::gaussRand(0., euc_noise), Sampler::gaussRand(0., euc_noise),
                     Sampler::gaussRand(0., euc_noise));

        pt1 += Vec3d(Sampler::gaussRand(0., euc_noise), Sampler::gaussRand(0., euc_noise),
                     Sampler::gaussRand(0., euc_noise));

        // form edge, with normals in varioius positions
        Vec3d nm0, nm1;
        nm0 << 0, i, 1;
        nm1 << 0, i, 1;
        nm0.normalize();
        nm1.normalize();

        // new edge with correct cohort for caching
        auto e = std::make_shared<Edge_V_V_GICP>();

        e->SetVertex(0, vp0);  // first viewpoint
        e->SetVertex(1, vp1);  // second viewpoint

        EdgeGICP meas;
        meas.pos0_ = pt0;
        meas.pos1_ = pt1;
        meas.normal0_ = nm0;
        meas.normal1_ = nm1;

        e->SetMeasurement(meas);
        meas = e->GetMeasurement();
        // use this for point-plane
        e->Information() = meas.prec0(0.01);

        // use this for point-point
        //    e->information().setIdentity();

        //    e->setRobustKernel(true);
        // e->setHuberWidth(0.01);

        optimizer.AddEdge(e);
    }

    // move second cam off of its true position
    auto vc = vp1;
    SE3 cam = vc->Estimate();
    cam.translation() = Vec3d(0, 0, 0.2);
    vc->SetEstimate(cam);

    optimizer.InitializeOptimization();
    optimizer.ComputeActiveErrors();
    LOG(INFO) << "Initial chi2 = " << optimizer.Chi2();

    optimizer.SetVerbose(true);

    optimizer.Optimize(5);

    LOG(INFO) << "Second vertex should be near 0,0,1";
    LOG(INFO) << vp0->Estimate().translation().transpose() << ", "
              << vp0->Estimate().unit_quaternion().coeffs().transpose();
    LOG(INFO) << vp1->Estimate().translation().transpose() << ", "
              << vp1->Estimate().unit_quaternion().coeffs().transpose();
    return 0;
}
