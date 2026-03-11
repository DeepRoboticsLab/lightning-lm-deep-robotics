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

DEFINE_int32(nodes_per_level, 100, "how many nodes per lap on the sphere");
DEFINE_int32(num_laps, 30, "how many times the robot travels around the sphere");
DEFINE_double(radius, 100.0, "radius of the sphere");
DEFINE_double(noise_translation, 0.1, "noise level for the translation");
DEFINE_double(noise_rotation, 0.001, "noise level for the rotation");

DEFINE_bool(incremental_mode, true, "should we set incremental mode");
DEFINE_int32(inc_vertex_size, -1, "max vertex size in inc mode, set -1 as unlimited size");

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

    OptimizerConfig config{AlgorithmType::LEVENBERG_MARQUARDT, miao::LinearSolverType::LINEAR_SOLVER_PCG, false};
    if (FLAGS_incremental_mode) {
        LOG(INFO) << "setting incremental mode";
    } else {
        LOG(INFO) << "not setting incremental mode";
    }

    config.incremental_mode_ = FLAGS_incremental_mode;
    config.max_vertex_size_ = FLAGS_inc_vertex_size;

    auto optimizer = SetupOptimizer(config);  // 增量形式的

    optimizer->SetVerbose(true);

    GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> transSampler;
    transSampler.setDistribution(transNoise);
    GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> rotSampler;
    rotSampler.setDistribution(rotNoise);

    auto gen_noisy_meas = [&](const SE3& meas) {
        Eigen::Quaterniond gtQuat = meas.so3().unit_quaternion();
        Vec3d gtTrans = meas.translation();

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
        return SE3(rot, trans);
    };

    int id = 0;
    std::shared_ptr<VertexSE3> prev = nullptr;

    std::map<int, SE3> motion_data;

    for (int f = 0; f < FLAGS_num_laps; ++f) {
        for (int n = 0; n < nodesPerLevel; ++n) {
            Eigen::AngleAxisd rotz(-M_PI + 2 * n * M_PI / nodesPerLevel, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd roty(-0.5 * M_PI + id * M_PI / (FLAGS_num_laps * nodesPerLevel),
                                   Eigen::Vector3d::UnitY());
            Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();

            Eigen::Isometry3d t;
            t = rot;
            t.translation() = t.linear() * Eigen::Vector3d(radius, 0, 0);
            motion_data.emplace(id++, SE3(t.matrix()));
        }
    }

    id = 0;

    for (int f = 0; f < FLAGS_num_laps; ++f) {
        for (int n = 0; n < nodesPerLevel; ++n) {
            auto v = std::make_shared<VertexSE3>();
            int idx = id;

            v->SetId(id++);
            v->SetEstimate(motion_data[idx]);

            vertices.push_back(v);
            optimizer->AddVertex(v);

            if (prev != nullptr) {
                SE3 motion = motion_data[prev->GetId()].inverse() * v->Estimate();
                auto e = std::make_shared<EdgeSE3>();
                e->SetVertex(0, prev);
                e->SetVertex(1, v);
                e->SetMeasurement(gen_noisy_meas(motion));
                e->SetInformation(information);

                odometryEdges.emplace_back(e);
                edges.emplace_back(e);

                optimizer->AddEdge(e);
            }

            prev = v;

            if (id % 10 == 0) {
                LOG(INFO) << "optimizing for " << id;
                Timer::Evaluate(
                    [&]() {
                        optimizer->InitializeOptimization();
                        optimizer->Optimize(5);
                    },
                    "optimization", true);
            }

            /// loop closure
            if (f == 0 || n == 1) {
                continue;
            }

            for (int nn = -1; nn <= 1; ++nn) {
                int idx = (f - 1) * nodesPerLevel + nn + n;
                if (idx < 0 || idx >= vertices.size()) {
                    continue;
                }

                if (FLAGS_inc_vertex_size > 0 && (id - idx) >= FLAGS_inc_vertex_size) {
                    continue;
                }

                auto from = vertices[idx];
                SE3 pose = motion_data[idx].inverse() * v->Estimate();
                auto e = std::make_shared<EdgeSE3>();
                e->SetVertex(0, from);
                e->SetVertex(1, v);
                e->SetMeasurement(gen_noisy_meas(pose));
                e->SetInformation(information);
                edges.emplace_back(e);
                optimizer->AddEdge(e);
            }
        }
    }

    Timer::Evaluate(
        [&]() {
            optimizer->InitializeOptimization();
            optimizer->Optimize(5);
        },
        "optimization", true);

    Timer::PrintAll();

    return 0;
}