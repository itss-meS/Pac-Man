#include "pacbot_core/kalman.hpp"
#include <Eigen/Dense>

namespace pacbot {

GhostEKF::GhostEKF(double dt) {
    // State: x, y, vx, vy
    x_ = StateVec::Zero();
    P_ = CovMat::Identity() * 10.0; // High initial uncertainty
    
    // Process Noise (Continuous White Noise Accel Model discretized)
    // Q = G * sigma_a^2 * G^T * dt (approx)
    double sigma_a = 0.5; // Accel noise magnitude
    double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
    Q_ << dt4/4, 0,     dt3/2, 0,
          0,     dt4/4, 0,     dt3/2,
          dt3/2, 0,     dt2,   0,
          0,     dt3/2, 0,     dt2;
    Q_ *= sigma_a * sigma_a;

    // Measurement Noise (Default)
    R_ << 0.1, 0,
          0,   0.1;

    // State Transition (Constant Velocity)
    F_ << 1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // Measurement Matrix (Observe x, y)
    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;
}

void GhostEKF::predict(double dt) {
    if (!initialized_) return;
    // Update F for variable dt
    F_(0, 2) = dt; F_(1, 3) = dt;
    
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

bool GhostEKF::update(const MeasVec& z, const MeasCov& R_meas) {
    if (!initialized_) {
        x_.head<2>() = z;
        x_.tail<2>().setZero();
        initialized_ = true;
        return true;
    }

    // Innovation
    MeasVec y = z - H_ * x_;
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_meas;
    
    // Kalman Gain
    Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();
    
    // Update
    x_ = x_ + K * y;
    P_ = (CovMat::Identity() - K * H_) * P_;
    
    return true;
}

void GhostEKF::initialize(const MeasVec& z) {
    x_.head<2>() = z;
    x_.tail<2>().setZero();
    P_.setIdentity(); P_ *= 1.0; // Reset certainty
    initialized_ = true;
}

std::vector<Vec2> GhostEKF::predict_trajectory(double horizon, double dt) const {
    std::vector<Vec2> traj;
    if (!initialized_) return traj;
    
    StateVec x_sim = x_;
    CovMat P_sim = P_; // Optional: propagate covariance for danger spread
    Eigen::Matrix4d F = F_; F(0,2)=dt; F(1,3)=dt;
    
    int steps = std::max(1, int(horizon / dt));
    traj.reserve(steps);
    
    for (int i = 0; i < steps; ++i) {
        traj.push_back(Vec2(x_sim(0), x_sim(1)));
        x_sim = F * x_sim;
        // P_sim = F * P_sim * F.transpose() + Q_; // If needed for danger ellipse
    }
    return traj;
}

} // namespace pacbot
