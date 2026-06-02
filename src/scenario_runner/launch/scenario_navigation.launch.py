"""
Scenario: heterogeneous fleet navigation.

Orchestrates the full repeatable run:
  - Gazebo with warehouse world
  - 3 Scout Minis with configured sensors and namespaces
  - 3 waypoint_navigator instances (one per robot)
  - telemetry_recorder writing to a timestamped experiment dir

When all navigators finish, the launch system will report completion;
recorder is stopped via Ctrl+C or by the navigators' shutdown signal.
"""

import os
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario_runner_share = get_package_share_directory('scenario_runner')
    scout_share = get_package_share_directory('scout_mini_description')
    recorder_share = get_package_share_directory('telemetry_recorder')

    scenario_config_path = os.path.join(
        scenario_runner_share, 'config', 'fleet_scenario.yaml'
    )
    fleet_launch = os.path.join(scout_share, 'launch', 'fleet.launch.py')
    recorder_launch = os.path.join(
        recorder_share, 'launch', 'recorder.launch.py'
    )
    fleet_topics_config = os.path.join(
        recorder_share, 'config', 'fleet_topics.yaml'
    )

    # Load scenario config to extract waypoints per robot
    with open(scenario_config_path, 'r') as f:
        scenario = yaml.safe_load(f)

    pd = scenario['scenario']['pd_params']

    # Start fleet (Gazebo + 3 robots)
    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fleet_launch),
    )

    # Start recorder with fleet topics config
    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recorder_launch),
        launch_arguments={
            'topics_config': fleet_topics_config,
            'experiment_name': LaunchConfiguration('experiment_name'),
        }.items(),
    )

    # One navigator per robot
    navigators = []
    for robot in scenario['robots']:
        waypoints_json = json.dumps(robot['waypoints'])
        navigator = Node(
            package='scenario_runner',
            executable='waypoint_navigator',
            namespace=robot['namespace'],
            name=f'navigator_{robot["id"]}',
            output='screen',
            parameters=[{
                'waypoints': waypoints_json,
                'tolerance': pd['tolerance'],
                'kp_linear': pd['kp_linear'],
                'kp_angular': pd['kp_angular'],
                'kd_angular': pd['kd_angular'],
                'max_linear': pd['max_linear'],
                'max_angular': pd['max_angular'],
                'control_rate_hz': pd['control_rate_hz'],
                'start_delay_s': pd['start_delay_s'],
            }],
        )
        navigators.append(navigator)

    return LaunchDescription([
        DeclareLaunchArgument(
            'experiment_name',
            default_value='fleet_navigation_baseline',
            description='Name of experiment dir under ~/field_robots_lab_experiments/'
        ),
        fleet,
        recorder,
        *navigators,
    ])
