#include "core/lio/eskf.hpp"

#include <iostream>
#include <stdexcept>

using namespace lightning;

static void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    NavState initial;
    initial.pos_ = Vec3d(1, 2, 3);
    initial.vel_ = Vec3d(2, -1, 0.5);
    initial.grav_ = S2(Vec3d(0, 0, -1));
    const double dt = 0.2;
    const Vec3d gyro(0, 0, 0.3);
    const auto zero_noise = ESKF::ProcessNoiseType::Zero().eval();

    ESKF cv(initial);
    cv.SetConstantVelocity(true);
    cv.Predict(dt, zero_noise, gyro, Vec3d(30, -40, 60));
    Check((cv.GetX().pos_ - initial.pos_ - initial.vel_ * dt).norm() < 1e-12,
          "Constant velocity must preserve vertical as well as horizontal motion");
    Check((cv.GetX().vel_ - initial.vel_).norm() < 1e-12, "Acceleration must not change the CV mean");
    Check((cv.GetX().rot_.inverse() * SO3::exp(gyro * dt)).log().norm() < 1e-12,
          "Gyroscope rotation must remain active");
    Check((cv.GetP().block<3, 3>(0, 0) - (1 + dt * dt) * Mat3d::Identity()).norm() < 1e-12,
          "Position covariance must follow the CV transition");
    Check((cv.GetP().block<3, 3>(0, 12) - dt * Mat3d::Identity()).norm() < 1e-12,
          "LiDAR position corrections must retain a path to velocity");
    Check(cv.GetP().block<3, 3>(12, 3).norm() < 1e-12 &&
              cv.GetP().block<3, 3>(12, 18).norm() < 1e-12 &&
              cv.GetP().block<3, 2>(12, 21).norm() < 1e-12,
          "CV covariance must not retain inertial acceleration/bias/gravity coupling");

    ESKF noisy(initial);
    noisy.SetConstantVelocity(true);
    auto process_noise = zero_noise;
    process_noise.block<3, 3>(3, 3) = 0.4 * Mat3d::Identity();
    process_noise.block<3, 3>(9, 9) = 100 * Mat3d::Identity();
    noisy.Predict(dt, process_noise, Vec3d::Zero(), Vec3d::Zero());
    Check((noisy.GetP().block<3, 3>(12, 12) - (1 + 0.4 * dt * dt) * Mat3d::Identity()).norm() < 1e-12,
          "Unmodelled acceleration must still increase velocity uncertainty");
    Check((noisy.GetP().block<3, 3>(18, 18) - Mat3d::Identity()).norm() < 1e-12,
          "Unused accelerometer bias must remain fixed in CV mode");

    ESKF inertial(initial);
    Check(!inertial.ConstantVelocity(), "The existing inertial model must remain the default");
    inertial.Predict(dt, zero_noise, Vec3d::Zero(), Vec3d(1, 0, -initial.grav_[2]));
    Check((inertial.GetX().vel_ - initial.vel_ - Vec3d(dt, 0, 0)).norm() < 1e-12,
          "Inertial prediction must still integrate acceleration");

    ESKF observed;
    observed.ChangeX(NavState());
    observed.SetConstantVelocity(true);
    ESKF::Options options;
    options.epsi_.setConstant(1e-8);
    options.use_aa_ = false;
    Vec3d measurement;
    options.lidar_obs_func_ = [&](NavState& state, ESKF::CustomObservationModel& obs) {
        obs.h_x_ = Eigen::MatrixXd::Zero(3, 12);
        obs.h_x_.block<3, 3>(0, 0).setIdentity();
        obs.residual_ = measurement - state.pos_;
        obs.lidar_residual_mean_ = obs.residual_.squaredNorm();
        obs.valid_ = true;
    };
    observed.Init(options);
    for (int i = 1; i <= 20; ++i) {
        observed.Predict(0.1, zero_noise, Vec3d::Zero(), Vec3d(0, 0, 100));
        measurement = Vec3d(0.1 * i, 0, 0.05 * i);
        observed.Update(ESKF::ObsType::LIDAR, 1e-4);
    }
    Check((observed.GetX().pos_ - measurement).norm() < 1e-3,
          "3D LiDAR observations must move the CV estimate up a ramp");
    Check((observed.GetX().vel_ - Vec3d(1, 0, 0.5)).norm() < 1e-3,
          "CV velocity must be learned from LiDAR, including its vertical component");
    Check(observed.GetP().allFinite(), "Covariance must stay finite");
    std::cout << "PASS: inertial propagation, CV mean/covariance, gyro rotation, and 3D ramp observations\n";
}
