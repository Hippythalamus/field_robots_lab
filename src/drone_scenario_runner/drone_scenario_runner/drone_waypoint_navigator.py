"""
PX4 offboard waypoint navigator for Scene 3.

Subscribes to /fmu/out/vehicle_local_position to track drone state.
Publishes /fmu/in/offboard_control_mode and /fmu/in/trajectory_setpoint
at 50 Hz. Sends /fmu/in/vehicle_command to arm and switch to offboard mode.

Lifecycle:
  1. Wait for first VehicleLocalPosition message (EKF converged).
  2. Stream OffboardControlMode + TrajectorySetpoint at 50 Hz for 1 second
     to satisfy PX4's offboard prerequisite.
  3. Send VehicleCommand: switch to offboard mode (1, 6).
  4. Send VehicleCommand: arm (400, 1).
  5. For each waypoint:
       - Publish trajectory_setpoint with target NED position.
       - When within tolerance, mark reached and move on.
  6. After last waypoint, send VehicleCommand: land (21).
  7. Wait for landing, then disarm and exit.

PX4 NED frame: x=North (forward), y=East, z=Down. So altitude h = -z.
"""

import math
import sys
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class DroneWaypointNavigator(Node):
    """Offboard waypoint mission for one PX4-controlled drone."""

    def __init__(self):
        super().__init__('drone_waypoint_navigator')

        self.declare_parameter('waypoints', '')  # JSON list of [x,y,z] (NED)
        self.declare_parameter('tolerance', 0.5)  # meters
        self.declare_parameter('cruise_speed', 3.0)  # m/s
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('mission_start_delay_s', 5.0)

        wp_str = self.get_parameter('waypoints').value
        self.tolerance = self.get_parameter('tolerance').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.mission_start_delay_s = self.get_parameter('mission_start_delay_s').value
        control_rate = self.get_parameter('control_rate_hz').value

        # Parse waypoints
        try:
            import json
            self.waypoints: List[Tuple[float, float, float]] = [
                tuple(p) for p in json.loads(wp_str)
            ]
        except Exception as e:
            self.get_logger().error(f'Bad waypoints param: {wp_str} ({e})')
            raise

        if not self.waypoints:
            self.get_logger().error('Empty waypoint list')
            raise ValueError('Empty waypoints')

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

        # State
        self.have_position = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_wp_idx = 0
        self.start_time = time.time()
        self.offboard_setpoint_counter = 0
        self.arming_sent = False
        self.offboard_sent = False
        self.landing_sent = False
        self.mission_complete = False
        self.landing_start_time = None
        self.nav_state = 0

        # QoS for PX4 — reliability=BEST_EFFORT, history=KEEP_LAST(10)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos
        )

        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._on_position,
            qos
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._on_status,
            qos
        )

        # Control loop
        self.timer = self.create_timer(1.0 / control_rate, self._control_step)

    def _on_position(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        if not self.have_position and msg.xy_valid and msg.z_valid:
            self.have_position = True
            self.get_logger().info(
                f'First valid position: x={self.current_x:.2f} '
                f'y={self.current_y:.2f} z={self.current_z:.2f}'
            )

    def _on_status(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state

    def _control_step(self):
        # Wait for valid position (EKF converged)
        if not self.have_position:
            return

        # Wait for startup grace period
        elapsed = time.time() - self.start_time
        if elapsed < self.mission_start_delay_s:
            return

        # Always publish offboard_control_mode + trajectory_setpoint at every step
        # — PX4 will failsafe to LOITER if setpoints stop coming.
        self._publish_offboard_control_mode()

        # Step 1: stream setpoints to "current position" for 10 cycles before
        # requesting offboard mode (PX4 prerequisite)
        if self.offboard_setpoint_counter < 10:
            self._publish_setpoint(self.current_x, self.current_y, self.current_z)
            self.offboard_setpoint_counter += 1
            return

        # Step 2: send offboard + arm commands once
        if not self.offboard_sent:
            self._send_offboard_command()
            self.offboard_sent = True
            self.get_logger().info('Offboard mode requested')
            time.sleep(0.1)

        if not self.arming_sent:
            self._send_arm_command()
            self.arming_sent = True
            self.get_logger().info('Arm command sent')
            time.sleep(0.1)
            return

        # Step 3: navigate through waypoints
        if self.current_wp_idx < len(self.waypoints):
            target_x, target_y, target_z = self.waypoints[self.current_wp_idx]

            # Publish setpoint for current target
            self._publish_setpoint(target_x, target_y, target_z)

            # Check if reached
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dz = target_z - self.current_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < self.tolerance:
                self.get_logger().info(
                    f'Reached waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}: '
                    f'({target_x}, {target_y}, {target_z}). distance={dist:.2f}'
                )
                self.current_wp_idx += 1
            return

        # Step 4: all waypoints done — land
        if not self.landing_sent:
            self.get_logger().info('All waypoints reached. Landing.')
            self._send_land_command()
            self.landing_sent = True
            self.landing_start_time = time.time()
            return

        # During landing, do NOT publish trajectory_setpoint — it conflicts
        # with AUTO_LAND mode. Only keep offboard_control_mode alive (already
        # published above) so PX4 doesn't reject our session.

        # Timer-based shutdown: wait 15s after land command, then exit.
        landing_elapsed = time.time() - self.landing_start_time
        if landing_elapsed > 15.0 and not self.mission_complete:
            self.mission_complete = True
            self.get_logger().info('Mission complete (landing window elapsed).')
            self.create_timer(1.0, self._shutdown_once)


    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0
        self.setpoint_pub.publish(msg)

    def _send_offboard_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0  # OFFBOARD
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _send_arm_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # arm
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _send_land_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _shutdown_once(self):
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DroneWaypointNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Ctrl+C')
    except Exception as e:
        print(f'Navigator error: {e}', file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
