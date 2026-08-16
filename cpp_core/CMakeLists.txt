# cpp_core/CMakeLists.txt
pybind11_add_module(pacbot_core
    src/pybind_wrapper.cpp
    src/astar.cpp
    src/kinematics.cpp
    src/kalman.cpp
)

target_include_directories(pacbot_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Link Eigen (Interface only)
target_link_libraries(pacbot_core PUBLIC Eigen3::Eigen)

# Set output dir to python package for direct import `import pacbot_core`
set_target_properties(pacbot_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../python/pacbot/core
    # Remove 'lib' prefix on Linux (pybind11 handles suffix)
    PREFIX ""
)

# Unit Tests
enable_testing()
add_executable(test_core tests/test_core.cpp)
target_link_libraries(test_core pacbot_core)
add_test(NAME CoreTests COMMAND test_core)
