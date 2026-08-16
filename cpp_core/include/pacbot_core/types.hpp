#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

namespace pacbot {

// --- Basic Geometry ---
struct Vec2 { double x, y; 
    Vec2() : x(0), y(0) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(double s) const { return {x*s, y*s}; }
    double norm() const { return std::hypot(x, y); }
};

struct Pose { 
    Vec2 pos; 
    double theta = 0.0; 
    Pose() = default;
    Pose(double x, double y, double t=0) : pos(x,y), theta(t) {}
};

// --- Grid Map ---
// 0 = Free, 1 = Wall, 255 = Unknown
using GridMap = std::vector<std::vector<uint8_t>>;
using DangerMap = std::vector<std::vector<double>>; // 0.0 - 1.0+

// --- Pathfinding ---
struct PathNode {
    int x, y;
    double g, h, f;
    int px, py; // Parent
    bool operator>(const PathNode& o) const { return f > o.f; }
};

struct PathResult {
    std::vector<std::array<int, 2>> path; // World coords [x, y]
    bool success = false;
    double cost = -1.0;
    int expansions = 0;
};

// --- Kinematics (Diff Drive) ---
struct WheelVelocities { double left, right; };
struct RobotState { Pose pose; double v_lin = 0; double v_ang = 0; };

struct KinematicConfig {
    double wheel_base = 0.5;       // meters (or cells)
    double wheel_radius = 0.05;    // meters
    double max_v_lin = 3.0;        // cells/sec
    double max_v_ang = 5.0;        // rad/sec
    double max_acc_lin = 8.0;      // cells/sec^2
    double max_acc_ang = 10.0;     // rad/sec^2
};

// --- Kalman Filter (Constant Velocity Model) ---
// State: [x, y, vx, vy]
using StateVec = Eigen::Vector4d;
using CovMat = Eigen::Matrix4d;
using MeasVec = Eigen::Vector2d; // [x, y]
using MeasCov = Eigen::Matrix2d;

} // namespace pacbot
