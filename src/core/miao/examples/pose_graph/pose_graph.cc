//
// Created by xiang on 24-6-14.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>

#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/robust_kernel_all.h"
#include "core/types/edge_se2.h"
#include "core/types/edge_se3.h"
#include "core/types/vertex_se2.h"
#include "core/types/vertex_se3.h"
#include "utils/timer.h"

DEFINE_string(g2o_file, "./dataset/pose_graph/sphere_bignoise_vertex3.g2o", "noise level for the rotation");
DEFINE_bool(is_3d_pose, true, "3d or 2d pose graph");
DEFINE_bool(with_rb, false, "add robust kernel");
DEFINE_bool(use_PCG, false, "use PCG solver");
DEFINE_int32(iterations, 100, "num of iterations");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning::miao;

    std::ifstream fin(FLAGS_g2o_file);
    if (!fin) {
        LOG(ERROR) << "cannot load file: " << FLAGS_g2o_file;
        return -1;
    }

    std::shared_ptr<Optimizer> optimizer = nullptr;
    OptimizerConfig config(
        AlgorithmType::LEVENBERG_MARQUARDT,
        FLAGS_use_PCG ? LinearSolverType::LINEAR_SOLVER_PCG : LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);

    if (FLAGS_is_3d_pose) {
        optimizer = SetupOptimizer<6, 3>(config);
    } else {
        optimizer = SetupOptimizer<3, 2>(config);
    }

    std::vector<std::shared_ptr<VertexSE3>> vertices_3d;
    std::vector<std::shared_ptr<EdgeSE3>> edges_3d;
    std::vector<std::shared_ptr<VertexSE2>> vertices_2d;
    std::vector<std::shared_ptr<EdgeSE2>> edges_2d;

    /// NOTE: g2o VertexSE3 与 miao::VertexSE3
    /// 的误差计算方式并不完全一致，rotation部分的误差和更新量相差一半，因此对rotation部分的协方差要除以4

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);

        if (line.empty()) {
            continue;
        }

        std::stringstream ss;
        ss << line;

        std::string name;
        ss >> name;

        if (name == "VERTEX_SE3:QUAT") {
            if (FLAGS_is_3d_pose == false) {
                LOG(FATAL) << "get 3d pose, but you set a 2D problem.";
            }

            int id = 0;
            double data[7] = {0};
            ss >> id;
            for (int i = 0; i < 7; ++i) {
                ss >> data[i];
            }

            auto v = std::make_shared<VertexSE3>();
            v->SetId(id);
            v->SetEstimate(SE3(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2])));

            optimizer->AddVertex(v);
            vertices_3d.emplace_back(v);
        } else if (name == "VERTEX_SE2") {
            if (FLAGS_is_3d_pose == true) {
                LOG(FATAL) << "get 2d pose, but you set a 3D problem.";
            }

            int id = 0;
            double data[3] = {0};
            ss >> id;
            for (int i = 0; i < 3; ++i) {
                ss >> data[i];
            }

            auto v = std::make_shared<VertexSE2>();
            v->SetId(id);
            v->SetEstimate(SE2(SO2(data[2]), Vec2d(data[0], data[1])));

            optimizer->AddVertex(v);
            vertices_2d.emplace_back(v);
        } else if (name == "EDGE_SE3:QUAT") {
            auto e = std::make_shared<EdgeSE3>();
            int v1 = 0, v2 = 0;
            ss >> v1 >> v2;

            e->SetVertex(0, optimizer->GetVertex(v1));
            e->SetVertex(1, optimizer->GetVertex(v2));

            double data[7] = {0};
            for (int i = 0; i < 7; ++i) {
                ss >> data[i];
            }
            SE3 meas(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2]));
            e->SetMeasurement(meas);

            // read info
            Mat6d info = Mat6d::Zero();
            for (int i = 0; i < info.rows(); ++i) {
                for (int j = i; j < info.cols(); ++j) {
                    ss >> info(i, j);
                    if (i != j) {
                        info(j, i) = info(i, j);
                    }
                }
            }

            info.block<3, 3>(3, 3) = 0.25 * info.block<3, 3>(3, 3);
            e->SetInformation(info);

            if (FLAGS_with_rb) {
                e->SetRobustKernel(std::make_shared<RobustKernelHuber>());
            }
            optimizer->AddEdge(e);
            edges_3d.emplace_back(e);
        } else if (name == "EDGE_SE2") {
            auto e = std::make_shared<EdgeSE2>();
            int v1 = 0, v2 = 0;
            ss >> v1 >> v2;

            e->SetVertex(0, optimizer->GetVertex(v1));
            e->SetVertex(1, optimizer->GetVertex(v2));

            double data[3] = {0};
            for (int i = 0; i < 3; ++i) {
                ss >> data[i];
            }
            SE2 meas(SO2(data[2]), Vec2d(data[0], data[1]));
            e->SetMeasurement(meas);

            // read info
            Mat3d info = Mat3d::Zero();
            for (int i = 0; i < info.rows(); ++i) {
                for (int j = i; j < info.cols(); ++j) {
                    ss >> info(i, j);
                    if (i != j) {
                        info(j, i) = info(i, j);
                    }
                }
            }

            e->SetInformation(info);

            if (FLAGS_with_rb) {
                e->SetRobustKernel(std::make_shared<RobustKernelHuber>());
            }
            optimizer->AddEdge(e);
            edges_2d.emplace_back(e);
        }
    }

    LOG(INFO) << "vert: " << optimizer->GetVertices().size() << ", edges: " << optimizer->GetEdges().size();
    optimizer->InitializeOptimization();
    optimizer->SetVerbose(true);

    Timer::Evaluate([&]() { optimizer->Optimize(FLAGS_iterations); }, "optimize");

    /// TODO: save
    LOG(INFO) << "saving graph to result.g2o";
    std::ofstream fout("./result.g2o");

    for (const auto& v : vertices_3d) {
        fout << "VERTEX_SE3:QUAT " << v->GetId() << " ";
        Vec3d t = v->Estimate().translation();
        Quatd q = v->Estimate().unit_quaternion();

        fout << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z()
             << std::endl;
    }

    for (const auto& v : vertices_2d) {
        fout << "VERTEX_SE2 " << v->GetId() << " ";
        Vec2d t = v->Estimate().translation();
        double theta = v->Estimate().so2().log();

        fout << t[0] << " " << t[1] << " " << theta << std::endl;
    }

    for (const auto& e : edges_3d) {
        fout << "EDGE_SE3:QUAT " << e->GetVertex(0)->GetId() << " " << e->GetVertex(1)->GetId() << " ";
        Vec3d t = e->GetMeasurement().translation();
        Quatd q = e->GetMeasurement().unit_quaternion();

        fout << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << " ";

        Mat6d info = e->Information();
        for (int i = 0; i < info.rows(); ++i) {
            for (int j = i; j < info.cols(); ++j) {
                fout << info(i, j) << " ";
            }
        }

        fout << std::endl;
    }

    for (const auto& e : edges_2d) {
        fout << "EDGE_SE2 " << e->GetVertex(0)->GetId() << " " << e->GetVertex(1)->GetId() << " ";
        Vec2d t = e->GetMeasurement().translation();
        double theta = e->GetMeasurement().so2().log();

        fout << t[0] << " " << t[1] << " " << theta << " ";

        auto info = e->Information();
        for (int i = 0; i < info.rows(); ++i) {
            for (int j = i; j < info.cols(); ++j) {
                fout << info(i, j) << " ";
            }
        }

        fout << std::endl;
    }

    fout.close();
    LOG(INFO) << "graph saved. ";

    miao::Timer::DumpIntoFile("./miao_pose.txt");
    Timer::PrintAll();

    return 0;
}