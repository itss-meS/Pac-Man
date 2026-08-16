#pragma once
#include "types.hpp"
#include <Eigen/Dense>

namespace pacbot {

class GhostEKF {
public:
    GhostEKF(double dt = 0.1); // Prediction timestep
    
    // Predict step (Constant Velocity Model)
    void predict(double dt);
    
    // Update step with measurement [x, y]
    // Returns true if update applied
    bool update(const MeasVec& z, const MeasCov& R_meas);
    
    // Get current estimated state [x, y, vx, vy]
    StateVec get_state() const { return x_; }
    CovMat get_covariance() const { return P_; }
    Pose get_pose() const { return Pose(x_(0), x_(1), std::atan2(x_(3), x_(2))); }
    
    // Initialize with first measurement
    void initialize(const MeasVec& z);

    // Predict future trajectory (for Danger Map)
    // horizon: seconds, dt: step
    std::vector<Vec2> predict_trajectory(double horizon, double dt) const;

private:
    StateVec x_; // [x, y, vx, vy]
    CovMat P_;
    CovMat Q_;   // Process Noise
    MeasCov R_;  // Measurement Noise (Default)
    Eigen::Matrix4d F_; // State Transition
    Eigen::Matrix<double, 2, 4> H_; // Measurement Matrix
    bool initialized_ = false;
};

} // namespace pacbot
