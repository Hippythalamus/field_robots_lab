import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('telemetry_recorder')
    default_config = os.path.join(pkg_share, 'config', 'scout_mini_topics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'topics_config',
            default_value=default_config,
            description='Path to topics YAML config'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser('~/field_robots_lab_experiments'),
            description='Where to write experiment data'
        ),
        DeclareLaunchArgument(
            'experiment_name',
            default_value='',
            description='Experiment name (auto-generated if empty)'
        ),
        Node(
            package='telemetry_recorder',
            executable='recorder_node',
            output='screen',
            parameters=[{
                'topics_config': LaunchConfiguration('topics_config'),
                'output_dir': LaunchConfiguration('output_dir'),
                'experiment_name': LaunchConfiguration('experiment_name'),
            }],
        ),
    ])
