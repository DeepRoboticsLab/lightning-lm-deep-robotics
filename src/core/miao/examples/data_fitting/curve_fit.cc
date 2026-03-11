//
// Created by xiang on 24-5-14.
//

#include <Eigen/Core>
#include <iostream>

#include "core/graph/base_unary_edge.h"
#include "core/graph/base_vec_vertex.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "utils/sampler.h"
#include "utils/timer.h"

/**
 * \brief the params, a, GetB, and lambda for a * exp(-lambda * t) + GetB
 */
class VertexParams : public miao::BaseVecVertex<3> {};

/**
 * \brief measurement for a point on the curve
 *
 * Here the measurement is the point which is lies on the curve.
 * The error function computes the difference between the curve
 * and the point.
 */
class EdgePointOnCurve : public miao::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePointOnCurve() = default;
    /// 计算误差
    void ComputeError() override {
        auto v = (VertexParams*)(GetVertex(0));
        Vec3d est = v->Estimate();

        double fval = est[0] * exp(-est[2] * measurement_[0]) + est[1];
        error_[0] = fval - measurement_[1];
    }

    void LinearizeOplus() override {
        auto v = (VertexParams*)(GetVertex(0));
        Vec3d est = v->Estimate();
        double exp_c0 = exp(-est[2] * measurement_[0]);

        jacobian_oplus_xi_[0] = exp_c0;
        jacobian_oplus_xi_[1] = 1;
        jacobian_oplus_xi_[2] = est[0] * (-est[2]) * exp_c0;
    }
};

DEFINE_int32(num_points, 100, "number of points");
DEFINE_int32(max_iterations, 10, "number of iterations");
DEFINE_bool(verbose, true, "Verbose output");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // generate random data
    miao::Sampler::seedRand();

    double a = 2.;
    double b = 0.4;
    double lambda = 0.2;
    std::vector<Eigen::Vector2d> points(FLAGS_num_points);
    for (int i = 0; i < FLAGS_num_points; ++i) {
        double x = miao::Sampler::uniformRand(0, 10);
        double y = a * exp(-lambda * x) + b;

        // add Gaussian noise
        y += miao::Sampler::gaussRand(0, 0.02);
        points[i].x() = x;
        points[i].y() = y;
    }

    // setup the solver
    miao::Optimizer optimizer;
    optimizer.SetVerbose(false);
    miao::SetupOptimizer<3, -1>(optimizer, miao::OptimizerConfig(miao::AlgorithmType::LEVENBERG_MARQUARDT));

    // build the optimization problem given the points
    // 1. add the parameter vertex
    auto params = std::make_shared<VertexParams>();
    params->SetId(0);
    params->SetEstimate(Eigen::Vector3d(1, 1, 1));  // some initial value for the params
    optimizer.AddVertex(params);

    // 2. add the points we measured to be on the curve
    for (int i = 0; i < FLAGS_num_points; ++i) {
        auto e = std::make_shared<EdgePointOnCurve>();
        e->SetInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->SetVertex(0, params);
        e->SetMeasurement(points[i]);

        optimizer.AddEdge(e);
    }

    // perform the optimization

    miao::Timer::Evaluate(
        [&]() {
            optimizer.InitializeOptimization();
            optimizer.SetVerbose(FLAGS_verbose);
            optimizer.Optimize(FLAGS_max_iterations);
        },
        "optimization");

    // print out the result
    LOG(INFO) << "Target curve";
    LOG(INFO) << "a * exp(-lambda * x) + b";
    LOG(INFO) << "Iterative least squares solution";
    LOG(INFO) << "a      = " << params->Estimate()(0);
    LOG(INFO) << "b      = " << params->Estimate()(1);
    LOG(INFO) << "lambda = " << params->Estimate()(2);
    LOG(INFO) << "real abc: " << a << ", " << b << ", " << lambda;

    miao::Timer::PrintAll();

    return 0;
}
