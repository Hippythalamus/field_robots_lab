"""
Mission orchestrator: symmetric synchronisation barriers for a
multi-robot scenario.

Two barriers, both implemented as latched Bool(True) signals so that
late subscribers (recorders, analyzers) still observe them:

  /scenario_runner/start
      Published once every robot has reported its first odometry
      message and a short settle period has elapsed. All navigators
      wait for this before driving, eliminating spawn-order drift
      from run-to-run timing.

  /scenario_runner/complete
      Published once every robot has published `True` on
      /robot_<ns>/mission_status — i.e. every navigator has finished
      its waypoint list. External runner scripts wait on this signal
      instead of grepping log files, which makes mission completion
      a deterministic ROS event rather than a text-pattern race.

Both barriers are observable in the recorded bag, giving a clean
mission-time framing (between start and complete) that downstream
analysis can use to align runs.
"""

import sys
import time
from typing import List, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')

        self.declare_parameter('robot_namespaces', ['robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('settle_s', 2.0)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('status_topic', 'mission_status')

        self.robot_namespaces: List[str] = list(
            self.get_parameter('robot_namespaces').value
        )
        self.settle_s: float = float(self.get_parameter('settle_s').value)
        odom_topic: str = self.get_parameter('odom_topic').value
        status_topic: str = self.get_parameter('status_topic').value

        self.get_logger().info(
            f'Orchestrating {len(self.robot_namespaces)} robots: '
            f'{self.robot_namespaces}'
        )

        # Publishers — both latched
        self.start_pub = self.create_publisher(
            Bool, '/scenario_runner/start', latched_qos()
        )
        self.complete_pub = self.create_publisher(
            Bool, '/scenario_runner/complete', latched_qos()
        )

        # --- Start barrier state ---
        self.ready: Set[str] = set()
        self.last_ready_time = None
        self.start_published = False

        # --- Complete barrier state ---
        self.done: Set[str] = set()
        self.complete_published = False

        # Subscriptions: odom from each robot, mission_status from each robot
        self._subs = []
        for ns in self.robot_namespaces:
            odom_full = f'/{ns}/{odom_topic}'
            status_full = f'/{ns}/{status_topic}'

            self._subs.append(self.create_subscription(
                Odometry,
                odom_full,
                lambda msg, n=ns: self._on_odom(n),
                10,
            ))
            # Status uses latched QoS so we don't miss the single True
            self._subs.append(self.create_subscription(
                Bool,
                status_full,
                lambda msg, n=ns: self._on_status(n, msg),
                latched_qos(),
            ))

        self._tick = self.create_timer(0.1, self._check_start)

    # --- Start barrier ---
    def _on_odom(self, ns: str):
        if ns in self.ready:
            return
        self.ready.add(ns)
        self.get_logger().info(
            f'Robot ready: {ns} ({len(self.ready)}/{len(self.robot_namespaces)})'
        )
        if len(self.ready) == len(self.robot_namespaces):
            self.last_ready_time = time.time()
            self.get_logger().info(
                f'All robots ready. Settling for {self.settle_s:.1f}s ...'
            )

    def _check_start(self):
        if self.start_published:
            return
        if self.last_ready_time is None:
            return
        if time.time() - self.last_ready_time >= self.settle_s:
            msg = Bool()
            msg.data = True
            self.start_pub.publish(msg)
            self.start_published = True
            self.get_logger().info(
                'START signal published on /scenario_runner/start'
            )

    # --- Complete barrier ---
    def _on_status(self, ns: str, msg: Bool):
        if not msg.data:
            return
        if ns in self.done:
            return
        self.done.add(ns)
        self.get_logger().info(
            f'Robot finished: {ns} ({len(self.done)}/{len(self.robot_namespaces)})'
        )
        if (len(self.done) == len(self.robot_namespaces)
                and not self.complete_published):
            self._publish_complete()

    def _publish_complete(self):
        msg = Bool()
        msg.data = True
        self.complete_pub.publish(msg)
        self.complete_published = True
        self.get_logger().info(
            'COMPLETE signal published on /scenario_runner/complete'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionOrchestrator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Ctrl+C')
    except Exception as e:
        print(f'mission_orchestrator error: {e}', file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
