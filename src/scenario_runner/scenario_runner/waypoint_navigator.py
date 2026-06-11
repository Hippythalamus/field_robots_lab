"""
Waypoint navigator with symmetric barriers and inspection logic.

Subscribes to odometry, publishes to cmd_vel. Both topics resolve
relative to the node's namespace; multiple instances run in parallel
by launching each in its own namespace.

Symmetric barriers
------------------
If use_start_barrier is True (default), the navigator waits for a
latched Bool(True) on /scenario_runner/start before driving. This
eliminates spawn-order drift between robots: all start within the
same control tick once mission_orchestrator releases the barrier.

When the mission finishes (all waypoints reached), the navigator
publishes Bool(True) on its own mission_status topic (resolved to
/<namespace>/mission_status). The orchestrator collects these and
publishes /scenario_runner/complete when all robots are done.

Inspection
----------
At designated waypoints the robot stops, dwells, generates a
deterministic synthetic reading (pressure, methane), evaluates
against thresholds, and publishes a structured inspection_event.
"""

import json
import math
import sys
import time
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def evaluate_decision(pressure: float, methane: float,
                      p_threshold: float, m_threshold: float) -> str:
    p_high = pressure > p_threshold
    m_high = methane > m_threshold
    if p_high and m_high:
        return 'escalate'
    if p_high or m_high:
        return 'flag'
    return 'continue'


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoints', '')
        self.declare_parameter('tolerance', 0.3)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('kd_angular', 0.3)
        self.declare_parameter('max_linear', 0.4)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_s', 3.0)
        self.declare_parameter('use_start_barrier', True)
        self.declare_parameter('start_barrier_topic', '/scenario_runner/start')
        self.declare_parameter('inspection_points', '{}')
        self.declare_parameter('inspection_dwell_s', 2.0)
        self.declare_parameter('pressure_threshold', 15.0)
        self.declare_parameter('methane_threshold', 100.0)

        wp_str = self.get_parameter('waypoints').value
        self.tolerance = self.get_parameter('tolerance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.start_delay_s = self.get_parameter('start_delay_s').value
        self.use_start_barrier = self.get_parameter('use_start_barrier').value
        barrier_topic = self.get_parameter('start_barrier_topic').value
        self.inspection_dwell_s = self.get_parameter('inspection_dwell_s').value
        self.p_threshold = self.get_parameter('pressure_threshold').value
        self.m_threshold = self.get_parameter('methane_threshold').value
        control_rate = self.get_parameter('control_rate_hz').value

        try:
            self.waypoints: List[List[float]] = [
                list(p) for p in json.loads(wp_str)
            ]
        except Exception as e:
            self.get_logger().error(f'Bad waypoints param: {wp_str} ({e})')
            raise
        if not self.waypoints:
            raise ValueError('Empty waypoints')

        try:
            raw = json.loads(self.get_parameter('inspection_points').value)
            self.inspection_points: Dict[int, Dict[str, Any]] = {
                int(k): v for k, v in raw.items()
            }
        except Exception as e:
            self.get_logger().error(f'Bad inspection_points param: {e}')
            self.inspection_points = {}

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints, '
            f'{len(self.inspection_points)} inspection points, '
            f'use_start_barrier={self.use_start_barrier}'
        )

        self.have_odom = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_wp_idx = 0
        self.prev_heading_error = 0.0
        self.start_time = time.time()
        self.mission_start_time: Optional[float] = None
        self.finished = False
        self.wp_start_time: Optional[float] = None
        self.barrier_released = not self.use_start_barrier

        self.inspection_active = False
        self.inspection_start_time: Optional[float] = None
        self.inspection_emitted = False

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.event_pub = self.create_publisher(String, 'inspection_event', 10)
        # Finish signal — latched so the orchestrator never misses it,
        # even if it subscribes a moment after we finish.
        self.status_pub = self.create_publisher(
            Bool, 'mission_status', latched_qos()
        )

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._on_odom, 20
        )

        if self.use_start_barrier:
            self.start_sub = self.create_subscription(
                Bool, barrier_topic, self._on_start_signal, latched_qos()
            )
            self.get_logger().info(
                f'Waiting for start signal on {barrier_topic}'
            )

        self.timer = self.create_timer(1.0 / control_rate, self._control_step)

    def _on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.have_odom:
            self.have_odom = True
            self.get_logger().info(
                f'First odom: x={self.current_x:.2f} y={self.current_y:.2f} '
                f'yaw={math.degrees(self.current_yaw):.1f}deg'
            )

    def _on_start_signal(self, msg: Bool):
        if msg.data and not self.barrier_released:
            self.barrier_released = True
            self.mission_start_time = time.time()
            self.get_logger().info('START barrier released — beginning mission')

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _emit_inspection(self, wp_idx: int, spec: Dict[str, Any]):
        tank_id = spec.get('tank_id', f'wp_{wp_idx}')
        pressure = float(spec.get('pressure', 0.0))
        methane = float(spec.get('methane', 0.0))
        decision = evaluate_decision(
            pressure, methane, self.p_threshold, self.m_threshold
        )
        mission_t = (time.time() - self.mission_start_time
                     if self.mission_start_time else
                     time.time() - self.start_time)
        event = {
            'event': 'inspection_completed',
            'waypoint_index': wp_idx,
            'tank_id': tank_id,
            'pressure_bar': pressure,
            'methane_ppm': methane,
            'pressure_threshold_bar': self.p_threshold,
            'methane_threshold_ppm': self.m_threshold,
            'decision': decision,
            'mission_time_s': mission_t,
            'pose': {
                'x': round(self.current_x, 3),
                'y': round(self.current_y, 3),
                'yaw': round(self.current_yaw, 3),
            },
        }
        msg = String()
        msg.data = json.dumps(event)
        self.event_pub.publish(msg)
        self.get_logger().info(
            f'INSPECTION {tank_id}: pressure={pressure:.1f} bar, '
            f'methane={methane:.1f} ppm -> {decision}'
        )

    def _emit_inspection_started(self, wp_idx: int, spec: Dict[str, Any]):
        mission_t = (time.time() - self.mission_start_time
                     if self.mission_start_time else
                     time.time() - self.start_time)
        event = {
            'event': 'inspection_started',
            'waypoint_index': wp_idx,
            'tank_id': spec.get('tank_id', f'wp_{wp_idx}'),
            'mission_time_s': mission_t,
        }
        msg = String()
        msg.data = json.dumps(event)
        self.event_pub.publish(msg)

    def _control_step(self):
        if not self.have_odom:
            return
        if not self.barrier_released:
            return
        if not self.use_start_barrier and self.mission_start_time is None:
            if time.time() - self.start_time < self.start_delay_s:
                return
            self.mission_start_time = time.time()
        if self.finished:
            return
        if self.current_wp_idx >= len(self.waypoints):
            self._finish()
            return
        if self.wp_start_time is None:
            self.wp_start_time = time.time()
            self.get_logger().info(
                f'Heading to waypoint {self.current_wp_idx + 1}/'
                f'{len(self.waypoints)}: {self.waypoints[self.current_wp_idx]}'
            )

        target_x, target_y = self.waypoints[self.current_wp_idx]

        if self.inspection_active:
            self._publish_zero()
            inspection_elapsed = time.time() - self.inspection_start_time
            if not self.inspection_emitted:
                if inspection_elapsed >= self.inspection_dwell_s / 2.0:
                    spec = self.inspection_points[self.current_wp_idx]
                    self._emit_inspection(self.current_wp_idx, spec)
                    self.inspection_emitted = True
            if inspection_elapsed >= self.inspection_dwell_s:
                self.current_wp_idx += 1
                self.wp_start_time = None
                self.inspection_active = False
                self.inspection_start_time = None
                self.inspection_emitted = False
                self.prev_heading_error = 0.0
            return

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.current_yaw)

        if distance < self.tolerance:
            wp_elapsed = time.time() - self.wp_start_time
            self.get_logger().info(
                f'Reached waypoint {self.current_wp_idx + 1} in '
                f'{wp_elapsed:.1f}s (dist {distance:.2f}m)'
            )
            if self.current_wp_idx in self.inspection_points:
                spec = self.inspection_points[self.current_wp_idx]
                self.inspection_active = True
                self.inspection_start_time = time.time()
                self.inspection_emitted = False
                self._emit_inspection_started(self.current_wp_idx, spec)
                self._publish_zero()
                return
            self.current_wp_idx += 1
            self.wp_start_time = None
            self.prev_heading_error = 0.0
            return

        dt = 1.0 / 20.0
        d_heading = (heading_error - self.prev_heading_error) / dt
        self.prev_heading_error = heading_error
        angular_z = (self.kp_angular * heading_error +
                     self.kd_angular * d_heading)
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))
        heading_gate = max(0.0, math.cos(heading_error))
        linear_x = self.kp_linear * distance * heading_gate
        linear_x = max(0.0, min(self.max_linear, linear_x))

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def _finish(self):
        self.finished = True
        self._publish_zero()
        ref = (self.mission_start_time
               if self.mission_start_time else self.start_time)
        elapsed = time.time() - ref
        self.get_logger().info(
            f'All {len(self.waypoints)} waypoints reached in '
            f'{elapsed:.1f}s. Publishing mission_status=True.'
        )
        status = Bool()
        status.data = True
        self.status_pub.publish(status)
        # Keep the node alive so the orchestrator definitely sees the
        # latched status message. The runner script issues a SIGINT
        # once /scenario_runner/complete fires, which exits cleanly.

    def destroy_node(self):
        # Stop the robot on shutdown
        try:
            self._publish_zero()
        except Exception:
            pass
        super().destroy_node()


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
