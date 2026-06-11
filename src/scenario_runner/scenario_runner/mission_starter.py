"""
Mission starter: synchronisation barrier for a multi-robot scenario.

Purpose
-------
In a multi-robot launch the robots spawn sequentially in Gazebo, which
means each navigator's notion of "start" drifts apart from the others
by however long the spawn queue takes. That drift contaminates any
repeatability measurement: the spawn lag is environment-dependent
(machine load, sim startup variance) rather than mission-deterministic.

This node removes the drift by acting as a barrier:
  1. Subscribe to the first odometry message from every expected robot.
  2. Wait an additional settle period after the last robot is ready, so
     all spawn-time transients have died down.
  3. Publish a single latched `Bool(True)` on /scenario_runner/start.

Navigators ignore their own start_delay timer and instead wait for that
message. All three start their mission within the same control tick,
making the run-to-run timeline directly comparable.

The latched (`durability=TRANSIENT_LOCAL`) publication also lets
late-joining tools (e.g. the recorder, an analyzer) see the start signal
even if they connect after it fires.
"""

import sys
import time
from typing import List, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class MissionStarter(Node):
    def __init__(self):
        super().__init__('mission_starter')

        self.declare_parameter('robot_namespaces', ['robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('settle_s', 2.0)
        self.declare_parameter('odom_topic', 'odom')

        self.robot_namespaces: List[str] = list(
            self.get_parameter('robot_namespaces').value
        )
        self.settle_s: float = float(self.get_parameter('settle_s').value)
        odom_topic: str = self.get_parameter('odom_topic').value

        self.get_logger().info(
            f'Waiting for odom from {len(self.robot_namespaces)} robots: '
            f'{self.robot_namespaces}'
        )

        # Latched publisher so late subscribers (recorder, analyzer) still
        # see the start signal even if they connect after it fires.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_pub = self.create_publisher(
            Bool, '/scenario_runner/start', latched_qos
        )

        self.ready: Set[str] = set()
        self.last_ready_time = None
        self.start_published = False

        # One subscription per robot
        self._subs = []
        for ns in self.robot_namespaces:
            topic = f'/{ns}/{odom_topic}'
            sub = self.create_subscription(
                Odometry,
                topic,
                lambda msg, n=ns: self._on_odom(n),
                10,
            )
            self._subs.append(sub)

        self._tick = self.create_timer(0.1, self._check)

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

    def _check(self):
        if self.start_published:
            return
        if self.last_ready_time is None:
            return
        if time.time() - self.last_ready_time >= self.settle_s:
            self._publish_start()

    def _publish_start(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self.start_published = True
        self.get_logger().info(
            'START signal published on /scenario_runner/start'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionStarter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Ctrl+C')
    except Exception as e:
        print(f'mission_starter error: {e}', file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
