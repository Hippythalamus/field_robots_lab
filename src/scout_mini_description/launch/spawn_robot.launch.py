"""Spawn one Scout Mini with given namespace and sensor config."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('scout_mini_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'scout_mini.urdf.xacro')

    robot_id = LaunchConfiguration('robot_id').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    use_imu = LaunchConfiguration('use_imu').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    lidar_samples = LaunchConfiguration('lidar_samples').perform(context)
    imu_rate = LaunchConfiguration('imu_rate').perform(context)

    # Namespace passed to xacro needs leading slash for ROS2 namespace semantics
    ns_for_plugins = f'/{namespace}' if namespace and not namespace.startswith('/') else namespace

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', urdf_path,
                ' namespace:=', ns_for_plugins,
                ' use_imu:=', use_imu,
                ' use_lidar:=', use_lidar,
                ' lidar_samples:=', lidar_samples,
                ' imu_rate:=', imu_rate,
            ]),
            value_type=str
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[robot_description],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace,
        arguments=[
            '-topic', 'robot_description',
            '-entity', f'scout_mini_{robot_id}',
            '-robot_namespace', namespace,
            '-x', x,
            '-y', y,
            '-z', z,
        ],
        output='screen',
    )

    return [robot_state_publisher, spawn_entity]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument('namespace', default_value='robot_0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_samples', default_value='360'),
        DeclareLaunchArgument('imu_rate', default_value='100'),
        OpaqueFunction(function=launch_setup),
    ])
