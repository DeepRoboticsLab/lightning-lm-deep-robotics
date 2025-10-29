//
// Created by xiang on 24-4-26.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/graph/base_unary_edge.h"
#include "core/graph/base_vertex.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "utils/sampler.h"
#include "utils/timer.h"

/**
 * \brief a circle located at x,y with radius r
 */
class VertexCircle : public miao::BaseVertex<3, Eigen::Vector3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexCircle() {}

    void OplusImpl(const double *update) override {
        Eigen::Vector3d::ConstMapType v(update);
        estimate_ += v;
    }
};

/**
 * \brief measurement for a point on the circle
 *
 * Here the measurement is the point which is on the circle.
 * The error function computes the distance of the point to
 * the center minus the radius of the circle.
 */
class EdgePointOnCircle : public miao::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePointOnCircle() {}

    /// 计算误差
    void ComputeError() override {
        auto v = (VertexCircle *)(vertices_[0]);
        auto circle = v->Estimate();
        const double radius = circle[2];
        const Vec2d center = circle.head<2>();

        // err = sqrt((mx-x)^2 + (my-y)^2) - r
        error_[0] = (measurement_ - center).norm() - radius;
    }

    /// 雅可比使用数值导数
    void LinearizeOplus() override {
        auto v = (VertexCircle *)(vertices_[0]);
        auto circle = v->Estimate();
        const Vec2d center = circle.head<2>();

        double e0 = (measurement_ - center).norm();
        jacobian_oplus_xi_[0] = (circle[0] - measurement_[0]) / e0;
        jacobian_oplus_xi_[1] = (circle[1] - measurement_[1]) / e0;
        jacobian_oplus_xi_[2] = -1;
    }
};

double errorOfSolution(int numPoints, const std::vector<Eigen::Vector2d> &points, const Eigen::Vector3d &circle) {
    Eigen::Vector2d center = circle.head<2>();
    double radius = circle(2);
    double error = 0.;
    for (int i = 0; i < numPoints; ++i) {
        double d = (points[i] - center).norm() - radius;
        error += d * d;
    }
    return error;
}

DEFINE_int32(num_points, 100, "number of points");
DEFINE_int32(max_iterations, 10, "number of iterations");
DEFINE_bool(verbose, true, "Verbose output");

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Vec2d center(4.0, 2.0);
    double radius = 2.0;
    std::vector<Vec2d> points(FLAGS_num_points);

    miao::Sampler::seedRand();
    for (int i = 0; i < FLAGS_num_points; ++i) {
        double r = miao::Sampler::gaussRand(radius, 0.05);
        double angle = miao::Sampler::uniformRand(0.0, 2.0 * M_PI);
        points[i].x() = center.x() + r * cos(angle);
        points[i].y() = center.y() + r * sin(angle);
    }

    auto optimizer = miao::SetupOptimizer<3, -1>(miao::OptimizerConfig(miao::AlgorithmType::GAUSS_NEWTON));

    // 1. add the circle vertex
    auto circle = std::make_shared<VertexCircle>();
    circle->SetId(0);
    circle->SetEstimate(Eigen::Vector3d(3.0, 3.0, 3.0));  // some initial value for the circle
    optimizer->AddVertex(circle);

    // 2. add the points we measured
    for (int i = 0; i < FLAGS_num_points; ++i) {
        auto e = std::make_shared<EdgePointOnCircle>();
        e->SetInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->SetVertex(0, circle);
        e->SetMeasurement(points[i]);
        optimizer->AddEdge(e);
    }

    // perform the optimization

    miao::Timer::Evaluate(
        [&]() {
            optimizer->InitializeOptimization();
            optimizer->SetVerbose(FLAGS_verbose);
            optimizer->Optimize(FLAGS_max_iterations);
        },
        "Miao optimizer");

    // print out the result
    LOG(INFO) << "Iterative least squares solution";
    LOG(INFO) << "center of the circle " << circle->Estimate().head<2>().transpose();
    LOG(INFO) << "radius of the cirlce " << circle->Estimate()(2);
    LOG(INFO) << "error " << errorOfSolution(FLAGS_num_points, points, circle->Estimate());

    miao::Timer::Evaluate(
        [&]() {
            Eigen::MatrixXd A(FLAGS_num_points, 3);
            Eigen::VectorXd b(FLAGS_num_points);

            /// NOTE: 这里的定义貌似不完全一样
            for (int i = 0; i < FLAGS_num_points; ++i) {
                A(i, 0) = -2 * points[i].x();
                A(i, 1) = -2 * points[i].y();
                A(i, 2) = 1;
                b(i) = -pow(points[i].x(), 2) - pow(points[i].y(), 2);
            }

            Vec3d solution = (A.transpose() * A).ldlt().solve(A.transpose() * b);
            // calculate the radius of the circle given the solution so far
            solution(2) = sqrt(pow(solution(0), 2) + pow(solution(1), 2) - solution(2));

            LOG(INFO) << "Linear least squares solution";
            LOG(INFO) << "center of the circle " << solution.head<2>().transpose();
            LOG(INFO) << "radius of the cirlce " << solution(2);
            LOG(INFO) << "error " << errorOfSolution(FLAGS_num_points, points, solution);
        },
        "GN Solver");

    miao::Timer::PrintAll();

    return 0;
}