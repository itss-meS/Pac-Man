#pragma once
#include "types.hpp"

namespace pacbot {

class DiffDriveController {
public:
    DiffDriveController(const KinematicConfig& cfg = KinematicConfig());
    
    // Updates internal state, returns wheel velocities to command
    // dt: seconds
    // target_pose: Next waypoint in path (cell coords + 0.5)
    WheelVelocities update(double dt, const Pose& target_pose);
    
    // Direct velocity command (for teleop / low level)
    WheelVelocities command_velocity(double v_lin, double v_ang);
    
    // Getters
    const RobotState& get_state() const { return state_; }
    void set_state(const RobotState& s) { state_ = s; }
    void reset_odom(const Pose& p) { state_.pose = p; state_.v_lin = 0; state_.v_ang = 0; }

private:
    KinematicConfig cfg_;
    RobotState state_;
    
    // PID Internals
    double int_lin_ = 0, int_ang_ = 0;
    double prev_err_lin_ = 0, prev_err_ang_ = 0;
    double kp_lin_ = 4.0, ki_lin_ = 0.1, kd_lin_ = 0.5;
    double kp_ang_ = 6.0, ki_ang_ = 0.0, kd_ang_ = 0.3;
    
    // Collision Resolution (Slide)
    Vec2 resolve_collision(const Vec2& next_pos, const GridMap& grid) const;
};

} // namespace pacbot
