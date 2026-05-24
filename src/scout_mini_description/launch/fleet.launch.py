"""
Scene 2: Heterogeneous fleet of three Scout Mini robots.

Robots:
  robot_0 : full sensor suite     (IMU 100Hz + Lidar 360 samples)
  robot_1 : reduced lidar         (IMU 100Hz + Lidar 180 samples)
  robot_2 : IMU only, no lidar    (IMU 100Hz)

Spawned in a triangle around the world center to avoid initial collisions.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('scout_mini_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_share, 'worlds', 'minimal_warehouse.world')
    spawn_launch = os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')

    # Start Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # Robot 0: full sensors (reference configuration)
    robot_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch),
        launch_arguments={
            'robot_id': '0',
            'namespace': 'robot_0',
            'prefix': 'r0_',
            'x': '-2.0', 'y': '-2.0', 'z': '0.15',
            'use_imu': 'true',
            'use_lidar': 'true',
            'lidar_samples': '360',
            'imu_rate': '100',
        }.items(),
    )

    # Robot 1: reduced lidar resolution
    robot_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch),
        launch_arguments={
            'robot_id': '1',
            'namespace': 'robot_1',
            'prefix': 'r1_',
            'x': '2.0', 'y': '-2.0', 'z': '0.15',
            'use_imu': 'true',
            'use_lidar': 'true',
            'lidar_samples': '180',
            'imu_rate': '100',
        }.items(),
    )

    # Robot 2: IMU only, no lidar
    robot_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch),
        launch_arguments={
            'robot_id': '2',
            'namespace': 'robot_2',
            'prefix': 'r2_',
            'x': '0.0', 'y': '2.0', 'z': '0.15',
            'use_imu': 'true',
            'use_lidar': 'false',
            'imu_rate': '100',
        }.items(),
    )

    return LaunchDescription([
        gazebo,
        robot_0,
        robot_1,
        robot_2,
    ])
