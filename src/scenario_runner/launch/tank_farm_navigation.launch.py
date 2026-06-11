"""
Scenario: tank farm patrol with Husky multi-robot fleet.

Orchestrates a deterministic repeatable run:
  - Gazebo with tank_farm.world
  - 3 Husky A200 robots in namespaces robot_1, robot_2, robot_3
  - mission_orchestrator: dual barrier
      * start barrier  /scenario_runner/start    — releases navigators
      * finish barrier /scenario_runner/complete — fires when all done
  - 3 waypoint_navigator instances, each waiting on the start barrier
    and publishing mission_status when finished
  - telemetry_recorder writing to a timestamped experiment dir
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
    husky_multi_share = get_package_share_directory('husky_multi')

    scenario_config_path = os.path.join(
        scenario_runner_share, 'config', 'tank_farm_scenario.yaml'
    )
    fleet_launch = os.path.join(
        husky_multi_share, 'launch', 'multi_husky.launch.py'
    )

    try:
        recorder_share = get_package_share_directory('telemetry_recorder')
        recorder_launch_path = os.path.join(
            recorder_share, 'launch', 'recorder.launch.py'
        )
        fleet_topics_config = os.path.join(
            recorder_share, 'config', 'tank_farm_topics.yaml'
        )
        has_recorder = os.path.exists(recorder_launch_path)
    except Exception:
        has_recorder = False
        recorder_launch_path = None
        fleet_topics_config = None

    with open(scenario_config_path, 'r') as f:
        scenario = yaml.safe_load(f)

    pd = scenario['scenario']['pd_params']
    inspection = scenario['scenario']['inspection']
    barrier = scenario['scenario'].get('start_barrier', {})
    barrier_enabled = barrier.get('enabled', True)
    barrier_topic = barrier.get('topic', '/scenario_runner/start')
    barrier_settle = float(barrier.get('settle_s', 2.0))

    robot_namespaces = [r['namespace'] for r in scenario['robots']]

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fleet_launch),
    )

    actions = [
        DeclareLaunchArgument(
            'experiment_name',
            default_value='tank_farm_patrol_baseline',
            description='Experiment dir under ~/field_robots_lab_experiments/'
        ),
        fleet,
    ]

    if has_recorder:
        recorder = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(recorder_launch_path),
            launch_arguments={
                'topics_config': fleet_topics_config,
                'experiment_name': LaunchConfiguration('experiment_name'),
            }.items(),
        )
        actions.append(recorder)

    # Mission orchestrator: handles both barriers (start + complete)
    orchestrator = Node(
        package='scenario_runner',
        executable='mission_orchestrator',
        name='mission_orchestrator',
        output='screen',
        parameters=[{
            'robot_namespaces': robot_namespaces,
            'settle_s': barrier_settle,
            'odom_topic': 'odom',
            'status_topic': 'mission_status',
        }],
    )
    actions.append(orchestrator)

    for robot in scenario['robots']:
        waypoints_json = json.dumps(robot['waypoints'])
        inspection_points_json = json.dumps(
            robot.get('inspection_points', {})
        )

        navigator = Node(
            package='scenario_runner',
            executable='waypoint_navigator',
            namespace=robot['namespace'],
            name=f'navigator_{robot["id"]}',
            output='screen',
            parameters=[{
                'waypoints': waypoints_json,
                'inspection_points': inspection_points_json,
                'inspection_dwell_s': inspection['dwell_s'],
                'pressure_threshold': inspection['pressure_threshold_bar'],
                'methane_threshold': inspection['methane_threshold_ppm'],
                'tolerance': pd['tolerance'],
                'kp_linear': pd['kp_linear'],
                'kp_angular': pd['kp_angular'],
                'kd_angular': pd['kd_angular'],
                'max_linear': pd['max_linear'],
                'max_angular': pd['max_angular'],
                'control_rate_hz': pd['control_rate_hz'],
                'start_delay_s': pd['start_delay_s'],
                'use_start_barrier': barrier_enabled,
                'start_barrier_topic': barrier_topic,
            }],
        )
        actions.append(navigator)

    return LaunchDescription(actions)
