//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "utils/reconstruction_diagnostics.h"
#include <chrono>
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
        options_.optimize_on_every_keyframe_ =
            YAML::LoadFile(yaml_path)["loop_closing"]["optimize_on_every_keyframe"].as<bool>(true);
    }

    if (!options_.online_mode_) {
        diagnostics_dir_ = diagnostics::Directory(yaml_path);
        if (!diagnostics_dir_.empty()) {
            diagnostics::Open(diagnostics_snapshots_, diagnostics_dir_ + "/snapshots.csv", "event,phase,id,timestamp,x,y,z,qx,qy,qz,qw");
            diagnostics::Open(diagnostics_candidates_, diagnostics_dir_ + "/candidates.csv", "event,id1,id2,stage,resolution,score,converged,iterations,target_points,source_points,x,y,z,qx,qy,qz,qw,reason");
            diagnostics::Open(diagnostics_edges_, diagnostics_dir_ + "/edges.csv", "event,phase,edge,id1,id2,level,active,chi2,robust_delta,x,y,z,qx,qy,qz,qw");
            diagnostics::Open(diagnostics_solves_, diagnostics_dir_ + "/solves.csv", "event,vertices,active_edges,iterations,seconds,cost");
            std::ofstream effective(diagnostics_dir_ + "/backend-effective.txt");
            effective << std::setprecision(17) << "loop_kf_gap=" << options_.loop_kf_gap_
                << "\nmin_id_interval=" << options_.min_id_interval_
                << "\nclosest_id_th=" << options_.closest_id_th_
                << "\nmax_range=" << options_.max_range_
                << "\nndt_score_th=" << options_.ndt_score_th_
                << "\nwith_height=" << options_.with_height_
                << "\noptimize_on_every_keyframe=" << options_.optimize_on_every_keyframe_
                << "\nmotion_trans_noise=" << options_.motion_trans_noise_
                << "\nmotion_rot_noise=" << options_.motion_rot_noise_
                << "\nloop_trans_noise=" << options_.loop_trans_noise_
                << "\nloop_rot_noise=" << options_.loop_rot_noise_
                << "\nrk_loop_th=" << options_.rk_loop_th_
                << "\nheight_noise=" << options_.height_noise_
                << "\nparallel=" << config.parallel_ << "\nincremental=" << config.incremental_mode_ << '\n';
        }
    }
    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    // 检测回环候选
    DetectLoopCandidates();

    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    }

    // 计算回环位姿
    ComputeLoopCandidates();

    // 位姿图优化
    PoseOptimization();

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();

    auto& kfs_mapping = all_keyframes_;
    Keyframe::Ptr check_first = nullptr;

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        return;
    }

    if (last_loop_kf_ && (cur_kf_->GetID() - last_loop_kf_->GetID()) <= options_.loop_kf_gap_) {
        LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        return;
    }

    for (auto kf : kfs_mapping) {
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            // 同条轨迹内，跳过一定的ID区间
            continue;
        }

        if (abs(int(kf->GetID() - cur_kf_->GetID())) < options_.closest_id_th_) {
            /// 在同一条轨迹中，如果间隔太近，就不考虑回环
            break;
        }

        Vec3d dt = kf->GetOptPose().translation() - cur_kf_->GetOptPose().translation();
        double t2d = dt.head<2>().norm();  // x-y distance
        double range_th = options_.max_range_;

        if (t2d < range_th) {
            LoopCandidate c(kf->GetID(), cur_kf_->GetID());
            c.Tij_ = kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();

            candidates_.emplace_back(c);
            check_first = kf;
        }
    }

    if (!candidates_.empty()) {
        last_loop_kf_ = cur_kf_;
    }

    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ComputeLoopCandidates() {
    if (candidates_.empty()) {
        return;
    }

    // 执行计算
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    // 保存成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : candidates_) {
        LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        if (diagnostics_candidates_.is_open()) {
            diagnostics_candidates_ << cur_kf_->GetID() << ',' << lc.idx1_ << ',' << lc.idx2_ << ",decision,0," << lc.ndt_score_ << ",0,0,0,0,";
            diagnostics::Pose(diagnostics_candidates_, lc.Tij_);
            diagnostics_candidates_ << ',' << (lc.ndt_score_ > options_.ndt_score_th_ ? "accepted_score" : "rejected_score") << '\n';
        }
        if (lc.ndt_score_ > options_.ndt_score_th_) {
            succ_candidates.emplace_back(lc);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    const int submap_idx_range = 40;
    auto kf1 = all_keyframes_.at(c.idx1_), kf2 = all_keyframes_.at(c.idx2_);

    auto build_submap = [this](int given_id, bool build_in_world) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
            int id = idx + given_id;
            if (id < 0 || id >= all_keyframes_.size()) {
                continue;
            }

            auto kf = all_keyframes_[id];
            CloudPtr cloud = kf->GetCloud();

            // RemoveGround(cloud, 0.1);

            if (cloud->empty()) {
                continue;
            }

            // 转到世界系下
            SE3 Twb = kf->GetLIOPose();

            if (!build_in_world) {
                Twb = all_keyframes_.at(given_id)->GetLIOPose().inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            *submap += *cloud_trans;
        }
        return submap;
    };

    auto submap_kf1 = build_submap(kf1->GetID(), true);

    CloudPtr submap_kf2 = kf2->GetCloud();

    if (submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        return;
    }

    Mat4f Tw2 = kf2->GetLIOPose().matrix().cast<float>();

    /// 不同分辨率下的匹配
    CloudPtr output(new PointCloudType);
    std::vector<double> res{10.0, 5.0, 2.0, 1.0};

    CloudPtr rough_map1, rough_map2;

    for (auto& r : res) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(40);

        ndt.setResolution(r);
        rough_map1 = VoxelGrid(submap_kf1, r * 0.1);
        rough_map2 = VoxelGrid(submap_kf2, r * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, Tw2);
        Tw2 = ndt.getFinalTransformation();

        c.ndt_score_ = ndt.getTransformationProbability();
        if (diagnostics_candidates_.is_open()) {
            diagnostics_candidates_ << cur_kf_->GetID() << ',' << c.idx1_ << ',' << c.idx2_ << ",ndt," << r << ',' << c.ndt_score_ << ','
                << ndt.hasConverged() << ',' << ndt.getFinalNumIteration() << ',' << rough_map1->size() << ',' << rough_map2->size() << ',';
            Mat4d diagnostic_T = Tw2.cast<double>();
            Quatd diagnostic_q(diagnostic_T.block<3,3>(0,0)); diagnostic_q.normalize();
            diagnostics::Pose(diagnostics_candidates_, SE3(diagnostic_q, Vec3d(diagnostic_T.block<3,1>(0,3))));
            diagnostics_candidates_ << ",registered_world_pose\n";
        }
    }

    Mat4d T = Tw2.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);

    c.Tij_ = kf1->GetLIOPose().inverse() * SE3(q, t);

    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_out.pcd", *output);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_tgt.pcd", *rough_map1);
}

void LoopClosing::PoseOptimization() {
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(cur_kf_->GetID());
    v->SetEstimate(cur_kf_->GetOptPose());

    optimizer_->AddVertex(v);
    kf_vert_.emplace_back(v);

    /// 上一个关键帧的运动约束
    /// TODO 3D激光最好是跟前面多个帧都有关联

    for (int i = 1; i < 3; i++) {
        int id = cur_kf_->GetID() - i;
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);

            SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            e->SetMeasurement(motion);
            e->SetInformation(info_motion_);
            optimizer_->AddEdge(e);
        }
    }

    if (options_.with_height_) {
        /// 高度约束
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(e);
    }

    /// 回环的约束
    for (auto& c : candidates_) {
        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, optimizer_->GetVertex(c.idx1_));
        e->SetVertex(1, optimizer_->GetVertex(c.idx2_));
        e->SetMeasurement(c.Tij_);
        e->SetInformation(info_loops_);

        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        optimizer_->AddEdge(e);
        edge_loops_.emplace_back(e);
    }

    if (optimizer_->GetEdges().empty()) {
        return;
    }

    // Odometry-only vertices already start at the relative-pose solution.
    if (!options_.optimize_on_every_keyframe_ && candidates_.empty()) {
        return;
    }

    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(false);

    if (!diagnostics_dir_.empty()) {
        DiagnosticSnapshot("before");
        DiagnosticEdges("before");
    }
    const auto solve_start = std::chrono::steady_clock::now();
    const int solve_iterations = optimizer_->Optimize(20);
    if (diagnostics_solves_.is_open()) {
        diagnostics_solves_ << cur_kf_->GetID() << ',' << kf_vert_.size() << ',' << optimizer_->ActiveEdges().size() << ',' << solve_iterations << ','
            << std::chrono::duration<double>(std::chrono::steady_clock::now()-solve_start).count() << ',' << optimizer_->ActiveRobustChi2() << '\n';
    }

    /// remove outliers
    int cnt_outliers = 0;
    for (auto& e : edge_loops_) {
        if (e->GetRobustKernel() == nullptr) {
            continue;
        }

        if (e->Chi2() > e->GetRobustKernel()->Delta()) {
            e->SetLevel(1);
            cnt_outliers++;
        } else {
            e->SetRobustKernel(nullptr);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "loop outliers: " << cnt_outliers << "/" << edge_loops_.size();
    }

    /// get results
    for (auto& vert : kf_vert_) {
        SE3 pose = vert->Estimate();
        all_keyframes_[vert->GetId()]->SetOptPose(pose);
    }

    if (!diagnostics_dir_.empty()) {
        DiagnosticSnapshot("after");
        DiagnosticEdges("after");
    }
    if (loop_cb_) {
        loop_cb_();
    }

    LOG(INFO) << "optimize finished, loops: " << edge_loops_.size();

    // LOG(INFO) << "lc: cur kf " << cur_kf_->GetID() << ", opt: " << cur_kf_->GetOptPose().translation().transpose()
    //           << ", lio: " << cur_kf_->GetLIOPose().translation().transpose();
}

void LoopClosing::DiagnosticSnapshot(const char* phase) {
    for (const auto& kf : all_keyframes_) {
        diagnostics_snapshots_ << cur_kf_->GetID() << ',' << phase << ',' << kf->GetID() << ',' << kf->GetState().timestamp_ << ',';
        diagnostics::Pose(diagnostics_snapshots_, kf->GetOptPose());
        diagnostics_snapshots_ << '\n';
    }
}
void LoopClosing::DiagnosticEdges(const char* phase) {
    const auto& active = optimizer_->ActiveEdges();
    for (const auto& e : edge_loops_) {
        e->ComputeError();
        diagnostics_edges_ << cur_kf_->GetID() << ',' << phase << ',' << e->GetInternalId() << ',' << e->GetVertex(0)->GetId() << ',' << e->GetVertex(1)->GetId() << ','
            << e->Level() << ',' << (std::find(active.begin(),active.end(),e.get()) != active.end()) << ',' << e->Chi2() << ','
            << (e->GetRobustKernel() ? e->GetRobustKernel()->Delta() : 0) << ',';
        diagnostics::Pose(diagnostics_edges_, e->GetMeasurement()); diagnostics_edges_ << '\n';
    }
}
void LoopClosing::ExportFinalDiagnostics() {
    if (diagnostics_dir_.empty()) return;
    DiagnosticSnapshot("final"); DiagnosticEdges("final");
    diagnostics_snapshots_.flush(); diagnostics_candidates_.flush(); diagnostics_edges_.flush(); diagnostics_solves_.flush();
}
}  // namespace lightning
