//
// Created by xiang on 24-6-4.
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
#include "core/types/edge_se3.h"
#include "core/types/vertex_se3.h"
#include "utils/sampler.h"
#include "utils/timer.h"

DEFINE_int32(nodes_per_level, 50, "how many nodes per lap on the sphere");
DEFINE_int32(num_laps, 50, "how many times the robot travels around the sphere");
DEFINE_double(radius, 100.0, "radius of the sphere");
DEFINE_double(noise_translation, 0.1, "noise level for the translation");
DEFINE_double(noise_rotation, 0.001, "noise level for the rotation");
DEFINE_bool(use_PCG, false, "use PCG instead of the Cholesky");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning::miao;

    // command line parsing
    int nodesPerLevel = FLAGS_nodes_per_level;
    double radius = FLAGS_radius;
    std::vector<double> noiseTranslation;
    std::vector<double> noiseRotation;

    for (int i = 0; i < 3; ++i) {
        noiseTranslation.emplace_back(FLAGS_noise_translation);
        noiseRotation.emplace_back(FLAGS_noise_rotation);
    }

    Mat3d transNoise = Mat3d::Zero();
    for (int i = 0; i < 3; ++i) {
        transNoise(i, i) = std::pow(noiseTranslation[i], 2);
    }

    Mat3d rotNoise = Mat3d::Zero();
    for (int i = 0; i < 3; ++i) {
        rotNoise(i, i) = std::pow(noiseRotation[i], 2);
    }

    Mat6d information = Mat6d::Zero();
    information.block<3, 3>(0, 0) = transNoise.inverse();
    information.block<3, 3>(3, 3) = rotNoise.inverse();

    std::vector<std::shared_ptr<VertexSE3>> vertices;
    std::vector<std::shared_ptr<EdgeSE3>> odometryEdges;
    std::vector<std::shared_ptr<EdgeSE3>> edges;

    auto optimizer = SetupOptimizer(OptimizerConfig(
        AlgorithmType::LEVENBERG_MARQUARDT,
        FLAGS_use_PCG ? miao::LinearSolverType::LINEAR_SOLVER_PCG : miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN,
        false));

    int id = 0;
    for (int f = 0; f < FLAGS_num_laps; ++f) {
        for (int n = 0; n < nodesPerLevel; ++n) {
            auto v = std::make_shared<VertexSE3>();
            v->SetId(id++);

            Eigen::AngleAxisd rotz(-M_PI + 2 * n * M_PI / nodesPerLevel, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd roty(-0.5 * M_PI + id * M_PI / (FLAGS_num_laps * nodesPerLevel),
                                   Eigen::Vector3d::UnitY());
            Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();

            Eigen::Isometry3d t;
            t = rot;
            t.translation() = t.linear() * Eigen::Vector3d(radius, 0, 0);
            v->SetEstimate(SE3(t.matrix()));

            vertices.push_back(v);
            optimizer->AddVertex(v);
        }
    }

    // generate odometry edges
    for (size_t i = 1; i < vertices.size(); ++i) {
        auto prev = vertices[i - 1];
        auto cur = vertices[i];

        SE3 t = prev->Estimate().inverse() * cur->Estimate();
        auto e = std::make_shared<EdgeSE3>();
        e->SetVertex(0, prev);
        e->SetVertex(1, cur);
        e->SetMeasurement(t);
        e->SetInformation(information);
        odometryEdges.emplace_back(e);
        edges.emplace_back(e);

        optimizer->AddEdge(e);
    }

    // generate loop closure edges
    for (int f = 1; f < FLAGS_num_laps; ++f) {
        for (int nn = 0; nn < nodesPerLevel; ++nn) {
            auto from = vertices[(f - 1) * nodesPerLevel + nn];
            for (int n = -1; n <= 1; ++n) {
                if (f == FLAGS_num_laps - 1 && n == 1) {
                    continue;
                }

                auto to = vertices[f * nodesPerLevel + nn + n];
                SE3 t = from->Estimate().inverse() * to->Estimate();
                auto e = std::make_shared<EdgeSE3>();
                e->SetVertex(0, from);
                e->SetVertex(1, to);
                e->SetMeasurement(t);
                e->SetInformation(information);
                edges.emplace_back(e);

                optimizer->AddEdge(e);
            }
        }
    }

    GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> transSampler;
    transSampler.setDistribution(transNoise);
    GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> rotSampler;
    rotSampler.setDistribution(rotNoise);

    // noise for all the edges
    for (auto& e : edges) {
        Eigen::Quaterniond gtQuat = e->GetMeasurement().so3().unit_quaternion();
        Vec3d gtTrans = e->GetMeasurement().translation();

        Vec3d quatXYZ = rotSampler.generateSample();
        double qw = 1.0 - quatXYZ.norm();
        if (qw < 0) {
            qw = 0.;
        }

        Eigen::Quaterniond rot(qw, quatXYZ.x(), quatXYZ.y(), quatXYZ.z());
        rot.normalize();
        Eigen::Vector3d trans = transSampler.generateSample();
        rot = gtQuat * rot;
        trans = gtTrans + trans;

        SE3 noisyMeasurement(rot, trans);
        e->SetMeasurement(noisyMeasurement);
    }

    Timer::Evaluate(
        [&]() {
            optimizer->InitializeOptimization();
            optimizer->SetVerbose(true);
            optimizer->Optimize(10);
        },
        "optimize");

    Timer::PrintAll();

    return 0;
}