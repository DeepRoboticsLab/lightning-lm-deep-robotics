//
// Created by xiang on 24-5-21.
//

#include <unordered_map>
#include <unordered_set>

#include "core/graph/base_binary_edge.h"
#include "core/graph/base_vertex.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/robust_kernel_all.h"
#include "core/types/vertex_pointxyz.h"
#include "core/types/vertex_se3.h"
#include "utils/sampler.h"
#include "utils/timer.h"

DEFINE_double(pixel_noise, 1.0, "pixel noise");
DEFINE_double(outlier_ratio, 0.0, "outlier ratio");
DEFINE_bool(robust_kernel, false, "use robust kernel");
DEFINE_bool(structure_only, false, "structure-only BA");
DEFINE_bool(dense, false, "use dense Solver");

/**
 * \brief Stereo camera vertex, derived from SE3 class.
 * Note that we use the actual pose of the vertex as its parameterization,
 * rather than the transform from RW to camera coords. Uses static vars for
 * camera params, so there is a single camera setup.
 */
class VertexSCam : public miao::VertexSE3 {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSCam() {}

    // capture the update function to reset aux transforms
    void OplusImpl(const double* update) override {
        miao::VertexSE3::OplusImpl(update);
        setAll();
    }

    // camera matrix and stereo baseline
    inline static Mat3d Kcam;
    inline static double baseline;

    // transformations
    Eigen::Matrix<double, 3, 4> w2n;  // transform from world to node coordinates
    Eigen::Matrix<double, 3, 4> w2i;  // transform from world to image coordinates

    // Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
    // calculating Jacobian wrt pose of a projection.
    Mat3d dRdx, dRdy, dRdz;

    // transforms
    static void transformW2F(Eigen::Matrix<double, 3, 4>& m, const Vec3d& trans, const Quat& qrot) {
        m.block<3, 3>(0, 0) = qrot.toRotationMatrix().transpose();
        m.col(3).setZero();  // make sure there's no translation
        Vec4d tt;
        tt.head(3) = trans;
        tt[3] = 1.0;
        m.col(3) = -m * tt;
    }

    static void transformF2W(Eigen::Matrix<double, 3, 4>& m, const Vec3d& trans, const Quat& qrot) {
        m.block<3, 3>(0, 0) = qrot.toRotationMatrix();
        m.col(3) = trans;
    }

    // set up camera matrix
    static void setKcam(double fx, double fy, double cx, double cy, double tx) {
        Kcam.setZero();
        Kcam(0, 0) = fx;
        Kcam(1, 1) = fy;
        Kcam(0, 2) = cx;
        Kcam(1, 2) = cy;
        Kcam(2, 2) = 1.0;
        baseline = tx;
    }

    // set transform from world to cam coords
    void setTransform() { w2n = estimate_.inverse().matrix().block<3, 4>(0, 0); }

    // Set up world-to-image projection matrix (w2i), assumes camera parameters
    // are filled.
    void setProjection() { w2i = Kcam * w2n; }

    // sets angle derivatives
    void setDr() {
        // inefficient, just for testing
        // use simple multiplications and additions for production code in
        // calculating dRdx,y,z for dS'*R', with dS the incremental change
        dRdx = dRidx * w2n.block<3, 3>(0, 0);
        dRdy = dRidy * w2n.block<3, 3>(0, 0);
        dRdz = dRidz * w2n.block<3, 3>(0, 0);
    }

    // set all aux transforms
    void setAll() {
        setTransform();
        setProjection();
        setDr();
    }

    // calculate stereo projection
    void mapPoint(Vec3d& res, const Vec3d& pt3) {
        Vec4d pt;
        pt.head<3>() = pt3;
        pt(3) = miao::cst(1.0);
        Vec3d p1 = w2i * pt;
        Vec3d p2 = w2n * pt;
        Vec3d pb(baseline, 0, 0);

        double invp1 = miao::cst(1.0) / p1(2);
        res.head<2>() = p1.head<2>() * invp1;

        // right camera px
        p2 = Kcam * (p2 - pb);
        res(2) = p2(0) / p2(2);
    }

    inline static Mat3d dRidx;
    inline static Mat3d dRidy;
    inline static Mat3d dRidz;
};

// stereo projection
// first two args are the measurement type, second two the connection classes
class Edge_XYZ_VSC : public miao::BaseBinaryEdge<3, Vector3, miao::VertexPointXYZ, VertexSCam> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge_XYZ_VSC() {}
    // return the error estimate as a 2-vector
    void ComputeError() override {
        // from <Point> to <Cam>
        auto point = (miao::VertexPointXYZ*)(vertices_[0]);
        auto cam = (VertexSCam*)(vertices_[1]);

        // calculate the projection
        Vector3 kp;
        cam->mapPoint(kp, point->Estimate());

        // error, which is backwards from the normal observed - calculated
        // _measurement is the measured projection
        error_ = kp - measurement_;
    }
};

class Sample {
   public:
    static int uniform(int from, int to) { return static_cast<int>(miao::Sampler::uniformRand(from, to)); }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning::miao;
    Optimizer optimizer;
    if (FLAGS_dense) {
        LOG(INFO) << "Use dense Solver";
        SetupOptimizer<6, 3>(optimizer, OptimizerConfig(AlgorithmType::LEVENBERG_MARQUARDT,
                                                        LinearSolverType::LINEAR_SOLVER_DENSE, true));
    } else {
        LOG(INFO) << "Use eigen sparse Solver";
        SetupOptimizer<6, 3>(optimizer, OptimizerConfig(AlgorithmType::LEVENBERG_MARQUARDT,
                                                        LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false));
    }

    // set up 500 points
    std::vector<Vec3d> true_points;
    for (size_t i = 0; i < 500; ++i) {
        true_points.emplace_back((Sampler::uniformRand(0., 1.) - 0.5) * 3, Sampler::uniformRand(0., 1.) - 0.5,
                                 Sampler::uniformRand(0., 1.) + 10);
    }

    Vec2d focal_length(500, 500);     // pixels
    Vec2d principal_point(320, 240);  // 640x480 image
    double baseline = 0.075;          // 7.5 cm baseline

    std::vector<SE3> true_poses;

    VertexSCam::setKcam(focal_length[0], focal_length[1], principal_point[0], principal_point[1], baseline);

    // set up 5 vertices, first 2 fixed
    int vertex_id = 0;
    for (size_t i = 0; i < 5; ++i) {
        Vec3d trans(i * 0.04 - 1., 0, 0);

        Eigen::Quaterniond q;
        q.setIdentity();
        SE3 pose(q, trans);

        auto v_se3 = std::make_shared<VertexSCam>();

        v_se3->SetId(vertex_id);
        v_se3->SetEstimate(pose);
        v_se3->setAll();  // set aux transforms

        if (i < 2) {
            v_se3->SetFixed(true);
        }

        optimizer.AddVertex(v_se3);
        true_poses.push_back(pose);
        vertex_id++;
    }

    int point_id = vertex_id;
    double sum_diff2 = 0;

    std::unordered_map<int, int> pointid_2_trueid;
    std::unordered_set<int> inliers;

    // add point projections to this vertex
    for (size_t i = 0; i < true_points.size(); ++i) {
        auto v_p = std::make_shared<VertexPointXYZ>();

        v_p->SetId(point_id);
        v_p->SetMarginalized(true);
        v_p->SetEstimate(true_points.at(i) +
                         Vec3d(Sampler::gaussRand(0., 1), Sampler::gaussRand(0., 1), Sampler::gaussRand(0., 1)));

        int num_obs = 0;

        for (size_t j = 0; j < true_poses.size(); ++j) {
            /// 计算是否落在相机范围内
            Vec3d z;
            std::dynamic_pointer_cast<VertexSCam>(optimizer.GetVertex(j))->mapPoint(z, true_points.at(i));

            if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
                ++num_obs;
            }
        }

        if (num_obs < 2) {
            continue;
        }

        optimizer.AddVertex(v_p);

        bool inlier = true;
        for (size_t j = 0; j < true_poses.size(); ++j) {
            Vec3d z;
            std::dynamic_pointer_cast<VertexSCam>(optimizer.GetVertex(j))->mapPoint(z, true_points.at(i));

            if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
                double sam = Sampler::uniformRand(0., 1.);
                if (sam < FLAGS_outlier_ratio) {
                    z = Vec3d(Sample::uniform(64, 640), Sample::uniform(0, 480), Sample::uniform(0, 64));  // disparity
                    z(2) = z(0) - z(2);                                                                    // px' now

                    inlier = false;
                }

                z += Vec3d(Sampler::gaussRand(0., FLAGS_pixel_noise), Sampler::gaussRand(0., FLAGS_pixel_noise),
                           Sampler::gaussRand(0., FLAGS_pixel_noise / 16.0));

                auto e = std::make_shared<Edge_XYZ_VSC>();

                e->SetVertex(0, v_p);
                e->SetVertex(1, optimizer.GetVertices()[j]);

                e->SetMeasurement(z);
                e->SetInformation(Mat3d::Identity());

                if (FLAGS_robust_kernel) {
                    e->SetRobustKernel(std::make_shared<RobustKernelHuber>());
                }

                optimizer.AddEdge(e);
            }
        }

        if (inlier) {
            inliers.insert(point_id);
            Vec3d diff = v_p->Estimate() - true_points[i];

            sum_diff2 += diff.dot(diff);
        }

        pointid_2_trueid.insert(std::make_pair(point_id, i));

        ++point_id;
    }

    Timer::Evaluate(
        [&]() {
            optimizer.InitializeOptimization();
            optimizer.SetVerbose(true);
            optimizer.Optimize(20);
        },
        "Optimize");

    LOG(INFO) << "Point error before optimisation (inliers only): " << sqrt(sum_diff2 / inliers.size());
    sum_diff2 = 0;

    for (auto it : pointid_2_trueid) {
        auto v_p = std::dynamic_pointer_cast<VertexPointXYZ>(optimizer.GetVertex(it.first));

        if (v_p == nullptr) {
            LOG(ERROR) << "Vertex " << it.first << "is not a PointXYZ!";
            exit(-1);
        }

        Vec3d diff = v_p->Estimate() - true_points[it.second];
        if (inliers.find(it.first) == inliers.end()) {
            continue;
        }

        sum_diff2 += diff.dot(diff);
    }

    LOG(INFO) << "Point error after optimisation (inliers only): " << sqrt(sum_diff2 / inliers.size());
    Timer::PrintAll();

    return 0;
}