# ros2_ws/src/pacbot_bringup/launch/sim.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # 2. Robot Description (URDF/Xacro)
    # 3. Spawn Entity
    # 4. Nav2 Bringup (Params from pacbot_nav2_config)
    # 5. PacBot Strategy Node (Python)
    # 6. Ghost Detector Node (C++ EKF)
    
    return LaunchDescription([gz_sim]) # Simplified
