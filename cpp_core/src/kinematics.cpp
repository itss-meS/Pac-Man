#include "pacbot_core/kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace pacbot {

DiffDriveController::DiffDriveController(const KinematicConfig& cfg) : cfg_(cfg) {}

pacbot::WheelVelocities DiffDriveController::update(double dt, const Pose& target) {
    // 1. Compute Errors (Pure Pursuit style lookahead)
    double dx = target.pos.x - state_.pose.pos.x;
    double dy = target.pos.y - state_.pose.pos.y;
    double dist_err = std::hypot(dx, dy);
    
    double target_angle = std::atan2(dy, dx);
    double angle_err = target_angle - state_.pose.theta;
    // Normalize [-pi, pi]
    angle_err = std::fmod(angle_err + M_PI, 2*M_PI) - M_PI;

    // 2. PID Linear
    int_lin_ = std::clamp(int_lin_ + dist_err * dt, -10.0, 10.0);
    double deriv_lin = (dist_err - prev_err_lin_) / dt;
    double v_cmd = kp_lin_ * dist_err + ki_lin_ * int_lin_ + kd_lin_ * deriv_lin;
    prev_err_lin_ = dist_err;

    // 3. PID Angular
    int_ang_ = std::clamp(int_ang_ + angle_err * dt, -5.0, 5.0);
    double deriv_ang = (angle_err - prev_err_ang_) / dt;
    double w_cmd = kp_ang_ * angle_err + ki_ang_ * int_ang_ + kd_ang_ * deriv_ang;
    prev_err_ang_ = angle_err;

    // 4. Constraints
    v_cmd = std::clamp(v_cmd, -cfg_.max_v_lin, cfg_.max_v_lin);
    w_cmd = std::clamp(w_cmd, -cfg_.max_v_ang, cfg_.max_v_ang);

    // Accel Limits
    double dv = v_cmd - state_.v_lin;
    double max_dv = cfg_.max_acc_lin * dt;
    if (std::abs(dv) > max_dv) v_cmd = state_.v_lin + std::copysign(max_dv, dv);
    
    double dw = w_cmd - state_.v_ang;
    double max_dw = cfg_.max_acc_ang * dt;
    if (std::abs(dw) > max_dw) w_cmd = state_.v_ang + std::copysign(max_dw, dw);

    state_.v_lin = v_cmd;
    state_.v_ang = w_cmd;

    // 5. Kinematics Integration (Dead Reckoning)
    // v = (vl + vr)/2, w = (vr - vl)/L
    double vl = v_cmd - (w_cmd * cfg_.wheel_base / 2.0);
    double vr = v_cmd + (w_cmd * cfg_.wheel_base / 2.0);
    
    // Integrate Pose
    state_.pose.theta += w_cmd * dt;
    state_.pose.pos.x += v_cmd * std::cos(state_.pose.theta) * dt;
    state_.pose.pos.y += v_cmd * std::sin(state_.pose.theta) * dt;
    state_.pose.theta = std::fmod(state_.pose.theta + M_PI, 2*M_PI) - M_PI;

    return {vl, vr};
}

pacbot::WheelVelocities DiffDriveController::command_velocity(double v_lin, double v_ang) {
    state_.v_lin = std::clamp(v_lin, -cfg_.max_v_lin, cfg_.max_v_lin);
    state_.v_ang = std::clamp(v_ang, -cfg_.max_v_ang, cfg_.max_v_ang);
    double vl = state_.v_lin - (state_.v_ang * cfg_.wheel_base / 2.0);
    double vr = state_.v_lin + (state_.v_ang * cfg_.wheel_base / 2.0);
    return {vl, vr};
}

} // namespace pacbot
