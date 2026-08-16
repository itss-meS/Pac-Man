cmake_minimum_required(VERSION 3.16)
project(pacbot_competition LANGUAGES CXX)

# Default to Release for performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization Flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -march=native -ffast-math")
endif()

# Find Python & pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
# pybind11 installed via pip (global) or apt
find_package(pybind11 REQUIRED) 

# Eigen3 (Required for Kalman)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add Subdirectory
add_subdirectory(cpp_core)

# Install Python Package in editable mode helper target
add_custom_target(install_python
    COMMAND ${Python3_EXECUTABLE} -m pip install -e ${CMAKE_SOURCE_DIR}/python --no-build-isolation
    COMMENT "Installing Python package in editable mode"
    VERBATIM
)
