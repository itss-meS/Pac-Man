# 🤖 PacBot Competition Stack
**Autonomous Maze Navigation | Ghost Evasion | Point Maximization | Real-Time Replanning**

> **Architecture:** C++ Core (Pathfinding, Control, Estimation) + Python Logic (Strategy, FSM, Sensors) + ROS 2 (Hardware Deploy)

---

## 🏁 Quick Start (Simulation Only)

### 1. Prerequisites
- **Ubuntu 22.04+** / WSL2 / Docker
- **Python 3.10+**
- **C++ Compiler** (GCC 11+ / Clang 14+)
- **CMake 3.16+**
- **Dependencies:** `sudo apt install libeigen3-dev python3-pybind11-dev python3-pip`

### 2. Build C++ Core (Required for Performance)
```bash
cd cpp_core
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
# Verify
cd build && python3 -c "import pacbot_core; print('C++ Core OK:', pacbot_core.astar_plan.__doc__)"
