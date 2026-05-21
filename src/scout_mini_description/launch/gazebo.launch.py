import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('scout_mini_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(pkg_share, 'urdf', 'scout_mini.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'minimal_warehouse.world')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_path]),
            value_type=str
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'scout_mini',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
