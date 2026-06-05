"""
Scene 3 mission: PX4 drone offboard waypoint flight + telemetry recording.

This launch starts:
  - MicroXRCEAgent on UDP port 8888 (PX4 <-> ROS2 bridge)
  - telemetry_recorder with drone_topics.yaml config
  - drone_waypoint_navigator with mission YAML

PX4 SITL must be running separately. Start it before this launch:
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=10025 make px4_sitl_default gazebo-classic
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drone_share = get_package_share_directory('drone_scenario_runner')
    recorder_share = get_package_share_directory('telemetry_recorder')

    mission_config = os.path.join(
        drone_share, 'config', 'pipeline_sweep_mission.yaml'
    )
    drone_topics_config = os.path.join(
        recorder_share, 'config', 'drone_topics.yaml'
    )
    recorder_launch = os.path.join(
        recorder_share, 'launch', 'recorder.launch.py'
    )

    # Micro-XRCE-DDS Agent (PX4 ↔ ROS2 bridge)
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        emulate_tty=True,
    )

    # Telemetry recorder
    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recorder_launch),
        launch_arguments={
            'topics_config': drone_topics_config,
            'experiment_name': LaunchConfiguration('experiment_name'),
        }.items(),
    )

    # Drone waypoint navigator
    navigator = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'drone_scenario_runner', 'drone_waypoint_navigator',
            '--ros-args', '--params-file', mission_config,
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'experiment_name',
            default_value='drone_pipeline_sweep_01',
            description='Experiment directory name under ~/field_robots_lab_experiments/'
        ),
        agent,
        recorder,
        navigator,
    ])
