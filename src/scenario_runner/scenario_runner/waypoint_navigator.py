"""
Waypoint navigator: drives a differential-drive robot through a list
of (x, y) waypoints using a PD controller.

Subscribes to odometry, publishes to cmd_vel. Both topics are resolved
relative to the node's namespace, so multiple instances can run in
parallel (one per robot) by launching each in its own namespace.

Lifecycle:
  1. Wait for first odom message to establish current pose.
  2. For each waypoint:
       a. Compute heading error and distance error.
       b. Apply PD control: angular_z = Kp_ang * heading + Kd_ang * d/dt(heading)
                            linear_x  = Kp_lin * distance, scaled down by heading error.
       c. When distance < tolerance, mark waypoint reached and move on.
  3. After last waypoint, publish zero velocity and exit cleanly.

Exits with status 0 on success.
"""

import math
import sys
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Parameters
        self.declare_parameter('waypoints', '')  # JSON-like list, e.g. "[[1.0, 0.0], [2.0, 2.0]]"
        self.declare_parameter('tolerance', 0.3)  # meters
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('kd_angular', 0.3)
        self.declare_parameter('max_linear', 0.4)   # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_s', 3.0)  # wait for sim to stabilize

        wp_str = self.get_parameter('waypoints').value
        self.tolerance = self.get_parameter('tolerance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.start_delay_s = self.get_parameter('start_delay_s').value
        control_rate = self.get_parameter('control_rate_hz').value

        # Parse waypoints (JSON-like string)
        try:
            import json
            self.waypoints: List[Tuple[float, float]] = [
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
        self.have_odom = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_wp_idx = 0
        self.prev_heading_error = 0.0
        self.start_time = time.time()
        self.finished = False
        self.wp_start_time = None

        # ROS interfaces (topics resolve via namespace from launch)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._on_odom, 20
        )

        # Control timer
        self.timer = self.create_timer(1.0 / control_rate, self._control_step)

    def _on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.have_odom:
            self.have_odom = True
            self.get_logger().info(
                f'First odom: x={self.current_x:.2f} y={self.current_y:.2f} '
                f'yaw={math.degrees(self.current_yaw):.1f}°'
            )

    def _control_step(self):
        # Wait for odom
        if not self.have_odom:
            return

        # Wait for startup grace period (let simulation stabilize)
        elapsed = time.time() - self.start_time
        if elapsed < self.start_delay_s:
            return

        if self.finished:
            return

        # All waypoints done?
        if self.current_wp_idx >= len(self.waypoints):
            self._finish()
            return

        # Initialize per-waypoint timer
        if self.wp_start_time is None:
            self.wp_start_time = time.time()
            self.get_logger().info(
                f'Heading to waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}: '
                f'{self.waypoints[self.current_wp_idx]}'
            )

        target_x, target_y = self.waypoints[self.current_wp_idx]

        # Distance and heading to waypoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.current_yaw)

        # Reached?
        if distance < self.tolerance:
            wp_elapsed = time.time() - self.wp_start_time
            self.get_logger().info(
                f'Reached waypoint {self.current_wp_idx + 1} in {wp_elapsed:.1f}s '
                f'(distance {distance:.2f}m)'
            )
            self.current_wp_idx += 1
            self.wp_start_time = None
            self.prev_heading_error = 0.0
            return

        # PD on angular, scaled P on linear
        dt = 1.0 / 20.0  # nominal control period
        d_heading = (heading_error - self.prev_heading_error) / dt
        self.prev_heading_error = heading_error

        angular_z = self.kp_angular * heading_error + self.kd_angular * d_heading
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        # Linear velocity gated by heading: don't drive forward if facing wrong way
        heading_gate = max(0.0, math.cos(heading_error))
        linear_x = self.kp_linear * distance * heading_gate
        linear_x = max(0.0, min(self.max_linear, linear_x))

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def _finish(self):
        self.finished = True
        # Publish zero velocity to stop the robot cleanly
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'All {len(self.waypoints)} waypoints reached in {elapsed:.1f}s. Exiting.'
        )
        # Schedule shutdown
        self.create_timer(0.5, self._shutdown_once)

    def _shutdown_once(self):
        # Stop again, then signal main to exit
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointNavigator()
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
