"""
Multi-Husky launch — spawns 3 Husky robots in a single Gazebo world.
Each robot lives in its own namespace (robot_1, robot_2, robot_3).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    GroupAction,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


# Robot configuration: name, spawn position (x, y, yaw)
ROBOTS = [
    {'name': 'robot_1', 'x': 0.0,  'y':  0.0, 'yaw': 0.0},
    {'name': 'robot_2', 'x': 0.0,  'y':  3.0, 'yaw': 0.0},
    {'name': 'robot_3', 'x': 0.0,  'y': -3.0, 'yaw': 0.0},
]


def make_robot_group(robot, gazebo_started):
    """Build all nodes for one robot in its namespace."""
    name = robot['name']
    prefix = f"{name}/"

    # Config file path for this robot
    config_file = PathJoinSubstitution(
        [FindPackageShare("husky_multi"), "config", f"control_{name}.yaml"]
    )

    # Generate URDF via xacro with prefix AND robot_namespace AND explicit config
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            f"prefix:={prefix}",
            " ",
            f"robot_namespace:={name}",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content,
                         "use_sim_time": True}

    # robot_state_publisher — in namespace
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=name,
        output="screen",
        parameters=[robot_description],
    )

    # Spawn entity in Gazebo at given position
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name=f"spawn_{name}",
        arguments=[
            "-entity", name,
            "-topic", f"/{name}/robot_description",
            "-x", str(robot['x']),
            "-y", str(robot['y']),
            "-Y", str(robot['yaw']),
        ],
        output="screen",
    )

    # No controller_manager spawners — we use diff_drive plugin directly
    return [rsp, spawn]


def generate_launch_description():
    # Gazebo resource path setup
    gz_resource_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/:',
            str(Path(get_package_share_directory('husky_description')).parent.resolve()),
        ]
    )

    # World argument (empty for now, will be tank_farm.world later)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution(
            [FindPackageShare('field_robots_worlds'), 'worlds', 'tank_farm.world']
        ),
        description='Gazebo world file path'
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen',
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    ld = LaunchDescription([world_arg])
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Stagger robot spawns — robot_2 after 8s, robot_3 after 16s
    # This avoids spawn_entity service congestion and controller_manager startup races
    for i, robot in enumerate(ROBOTS):
        actions = make_robot_group(robot, gzserver)
        if i == 0:
            for a in actions:
                ld.add_action(a)
        else:
            # Delay subsequent robots
            ld.add_action(TimerAction(period=float(i * 8), actions=actions))

    return ld
