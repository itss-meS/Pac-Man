#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h> // Auto-convert Eigen <-> Numpy

#include "pacbot_core/astar.hpp"
#include "pacbot_core/kinematics.hpp"
#include "pacbot_core/kalman.hpp"

namespace py = pybind11;
using namespace pacbot;

// --- Helper: Numpy -> Vector Conversion (Zero Copy if C-contiguous) ---
static GridMap numpy_to_grid(py::array_t<uint8_t> arr) {
    py::buffer_info buf = arr.request();
    if (buf.ndim != 2) throw std::runtime_error("Grid must be 2D");
    GridMap grid(buf.shape[0], std::vector<uint8_t>(buf.shape[1]));
    // Fast memcpy row by row
    for (size_t i = 0; i < buf.shape[0]; ++i) {
        std::memcpy(grid[i].data(), 
                    (uint8_t*)buf.ptr + i * buf.strides[0], 
                    buf.shape[1] * sizeof(uint8_t));
    }
    return grid;
}

static DangerMap numpy_to_danger(py::array_t<double> arr) {
    if (arr.size() == 0) return {};
    py::buffer_info buf = arr.request();
    if (buf.ndim != 2) throw std::runtime_error("Danger map must be 2D");
    DangerMap dm(buf.shape[0], std::vector<double>(buf.shape[1]));
    for (size_t i = 0; i < buf.shape[0]; ++i) {
        std::memcpy(dm[i].data(), 
                    (double*)buf.ptr + i * buf.strides[0] / sizeof(double), 
                    buf.shape[1] * sizeof(double));
    }
    return dm;
}

// --- Bindings ---

PYBIND11_MODULE(pacbot_core, m) {
    m.doc() = "PacBot High-Performance C++ Core";

    // Types
    py::class_<Vec2>(m, "Vec2")
        .def(py::init<double, double>())
        .def_readwrite("x", &Vec2::x)
        .def_readwrite("y", &Vec2::y)
        .def("__repr__", [](const Vec2& v) { return "<Vec2 " + std::to_string(v.x) + ", " + std::to_string(v.y) + ">"; });

    py::class_<Pose>(m, "Pose")
        .def(py::init<double, double, double>())
        .def_readwrite("pos", &Pose::pos)
        .def_readwrite("theta", &Pose::theta);

    py::class_<KinematicConfig>(m, "KinematicConfig")
        .def(py::init<>())
        .def_readwrite("wheel_base", &KinematicConfig::wheel_base)
        .def_readwrite("max_v_lin", &KinematicConfig::max_v_lin)
        .def_readwrite("max_v_ang", &KinematicConfig::max_v_ang)
        .def_readwrite("max_acc_lin", &KinematicConfig::max_acc_lin)
        .def_readwrite("max_acc_ang", &KinematicConfig::max_acc_ang);

    py::class_<WheelVelocities>(m, "WheelVelocities")
        .def_readwrite("left", &WheelVelocities::left)
        .def_readwrite("right", &WheelVelocities::right);

    py::class_<RobotState>(m, "RobotState")
        .def_readwrite("pose", &RobotState::pose)
        .def_readwrite("v_lin", &RobotState::v_lin)
        .def_readwrite("v_ang", &RobotState::v_ang);

    // A* Planner
    m.def("astar_plan", [](py::array_t<uint8_t> grid, 
                           std::array<int, 2> start, 
                           std::array<int, 2> goal,
                           py::array_t<double> danger_map, // Optional
                           double danger_weight,
                           int max_expansions) {
        
        GridMap g = numpy_to_grid(grid);
        DangerMap dm = numpy_to_danger(danger_map);
        
        PathResult res = astar_plan(g, start, goal, dm.empty() ? nullptr : &dm, danger_weight, max_expansions);
        
        // Convert path to Python List of Tuples
        py::list py_path;
        for (auto& p : res.path) py_path.append(py::make_tuple(p[0], p[1]));
        
        return py::dict(
            "path"_a = py_path,
            "success"_a = res.success,
            "cost"_a = res.cost,
            "expansions"_a = res.expansions
        );
    }, "Fast A* Pathfinding with Dynamic Danger Costs", 
    py::arg("grid"), py::arg("start"), py::arg("goal"), 
    py::arg("danger_map") = py::array_t<double>(), // Default empty
    py::arg("danger_weight") = 10.0, 
    py::arg("max_expansions") = 10000);

    // Kinematics Controller
    py::class_<DiffDriveController>(m, "DiffDriveController")
        .def(py::init<KinematicConfig>())
        .def("update", &DiffDriveController::update, 
             "Step simulation dt seconds towards target pose. Returns wheel velocities.")
        .def("command_velocity", &DiffDriveController::command_velocity)
        .def("get_state", &DiffDriveController::get_state, py::return_value_policy::reference)
        .def("set_state", &DiffDriveController::set_state)
        .def("reset_odom", &DiffDriveController::reset_odom);

    // Kalman Filter
    py::class_<GhostEKF>(m, "GhostEKF")
        .def(py::init<double>(), py::arg("dt")=0.1)
        .def("predict", &GhostEKF::predict)
        .def("update", &GhostEKF::update)
        .def("initialize", &GhostEKF::initialize)
        .def("get_state", &GhostEKF::get_state)
        .def("get_covariance", &GhostEKF::get_covariance)
        .def("get_pose", &GhostEKF::get_pose)
        .def("predict_trajectory", &GhostEKF::predict_trajectory);
}
