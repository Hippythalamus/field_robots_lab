"""
Diablo-style visual marker coordinator.

Listens to inspection_event topics from primary robot (default robot_1) and
spawns Gazebo entities for visual storytelling:

  - On `inspection_started`: spawn a glowing reticle ring on the ground
    around the inspected tank.
  - On `inspection_completed`: spawn a vertical "loot beam" over the tank,
    colour-coded by the decision (white = continue, amber = flag, red =
    escalate).

Both markers persist until /scenario_runner/complete fires, then remain on
the field — the final mission view shows all six beams above the inspected
tanks, communicating the result pattern at a glance.

Parameters:
  - robot_namespace        : robot to follow (default 'robot_1')
  - mesh_dir               : absolute path to STL meshes directory
                             (reticle.stl, beam.stl).
                             If empty, resolved from
                             share/diablo_visuals/meshes via ament_index.
  - tank_positions         : flat list of 12 floats — tank centres x,y
                             (defaults to tank_1..6 in the standard layout).
"""

import json
import os
import sys
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool
from gazebo_msgs.srv import SpawnEntity

from ament_index_python.packages import get_package_share_directory


# Decision -> RGB triple in [0, 1]
DECISION_COLORS = {
    'continue': (1.0, 1.0, 1.0),    # white
    'flag':     (1.0, 0.65, 0.0),   # amber
    'escalate': (1.0, 0.05, 0.05),  # red
}


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class DiabloVisuals(Node):
    def __init__(self):
        super().__init__('diablo_visuals')

        # Resolve package share directory once
        self.pkg_share = get_package_share_directory('diablo_visuals')
        default_mesh_dir = os.path.join(self.pkg_share, 'meshes')
        self.models_dir = os.path.join(self.pkg_share, 'models')

        self.declare_parameter('robot_namespace', 'robot_1')
        self.declare_parameter('mesh_dir', default_mesh_dir)
        self.declare_parameter('tank_positions', [
            16.0,  12.0,   # tank_1
            28.0,  12.0,   # tank_2
            40.0,  12.0,   # tank_3
            16.0, -12.0,   # tank_4
            28.0, -12.0,   # tank_5
            40.0, -12.0,   # tank_6
        ])

        self.robot_ns = self.get_parameter('robot_namespace').value
        self.mesh_dir = self.get_parameter('mesh_dir').value

        if not os.path.isdir(self.mesh_dir):
            raise RuntimeError(
                f'mesh_dir does not exist: {self.mesh_dir!r}')
        if not os.path.isdir(self.models_dir):
            raise RuntimeError(
                f'models dir does not exist: {self.models_dir!r}')

        # Parse tank positions
        raw = list(self.get_parameter('tank_positions').value)
        if len(raw) % 2 != 0 or len(raw) < 12:
            raise RuntimeError(
                f'tank_positions must be a flat list of 12+ floats; got {raw}')
        self.tank_positions: Dict[str, List[float]] = {}
        for i in range(6):
            tank_id = f'tank_{i + 1}'
            self.tank_positions[tank_id] = [raw[2 * i], raw[2 * i + 1]]

        self.get_logger().info(
            f'Tracking {self.robot_ns}, mesh_dir={self.mesh_dir}, '
            f'models_dir={self.models_dir}, '
            f'tanks={list(self.tank_positions.keys())}')

        # Load SDF templates
        self._reticle_template = self._load_model('reticle.sdf')
        self._beam_template = self._load_model('beam.sdf')

        # Track spawned entities to avoid duplicates
        self.spawned: set = set()

        # Service client for spawning
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        # Subscriptions
        topic = f'/{self.robot_ns}/inspection_event'
        self.create_subscription(String, topic, self._on_event, 20)
        self.create_subscription(
            Bool, '/scenario_runner/complete', self._on_complete, latched_qos())
        self.get_logger().info(f'Subscribed to {topic}')

        # Queue events that arrive before spawn service is up
        self._pending: List[dict] = []
        self._spawn_ready = False
        self._spawn_check_timer = self.create_timer(
            0.5, self._check_spawn_service)

    def _load_model(self, filename: str) -> str:
        """Load an SDF template from the package models directory."""
        path = os.path.join(self.models_dir, filename)
        with open(path, 'r') as f:
            return f.read()

    def _check_spawn_service(self):
        if self._spawn_ready:
            return
        if self.spawn_cli.service_is_ready():
            self._spawn_ready = True
            self.get_logger().info('/spawn_entity service is ready')
            for ev in self._pending:
                self._handle_event(ev)
            self._pending.clear()

    def _on_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Bad inspection event JSON: {e}')
            return

        if self._spawn_ready:
            self._handle_event(ev)
        else:
            self._pending.append(ev)

    def _handle_event(self, ev: dict):
        event_type = ev.get('event')
        tank_id = ev.get('tank_id')
        if tank_id not in self.tank_positions:
            self.get_logger().warn(
                f'unknown tank_id {tank_id}, available: '
                f'{list(self.tank_positions.keys())}')
            return

        x, y = self.tank_positions[tank_id]

        if event_type == 'inspection_started':
            entity_name = f'reticle_{tank_id}'
            if entity_name in self.spawned:
                return
            self._spawn_reticle(entity_name, x, y)

        elif event_type == 'inspection_completed':
            decision = ev.get('decision', 'continue')
            color = DECISION_COLORS.get(decision, (0.7, 0.7, 0.7))
            entity_name = f'beam_{tank_id}'
            if entity_name in self.spawned:
                return
            self._spawn_beam(entity_name, x, y, color, decision)

    def _spawn_reticle(self, entity_name: str, x: float, y: float):
        xml = self._reticle_template.replace('MESH_DIR', self.mesh_dir)
        self._spawn(entity_name, xml, x, y, z=0.0)
        self.get_logger().info(f'Spawned reticle at {entity_name}')

    def _spawn_beam(self, entity_name: str, x: float, y: float,
                    color: tuple, decision: str):
        r, g, b = color
        xml = (self._beam_template
               .replace('MESH_DIR', self.mesh_dir)
               .replace('COLOR_R', f'{r:.3f}')
               .replace('COLOR_G', f'{g:.3f}')
               .replace('COLOR_B', f'{b:.3f}'))
        # Beam starts at top of tank (~6m) so it appears to rise from the tank
        self._spawn(entity_name, xml, x, y, z=6.5)
        self.get_logger().info(
            f'Spawned beam {entity_name} (decision={decision})')

    def _spawn(self, entity_name: str, xml: str,
               x: float, y: float, z: float):
        req = SpawnEntity.Request()
        req.name = entity_name
        req.xml = xml
        req.robot_namespace = ''
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        future = self.spawn_cli.call_async(req)
        future.add_done_callback(
            lambda f, n=entity_name: self._on_spawn_done(n, f))
        # Mark eagerly so duplicates queued in pending are not re-spawned
        self.spawned.add(entity_name)

    def _on_spawn_done(self, entity_name: str, future):
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn(
                    f'Spawn failed for {entity_name}: '
                    f'{getattr(result, "status_message", "no result")}')
                self.spawned.discard(entity_name)
        except Exception as e:
            self.get_logger().warn(
                f'Spawn callback error for {entity_name}: {e}')
            self.spawned.discard(entity_name)

    def _on_complete(self, msg: Bool):
        if msg.data:
            self.get_logger().info(
                f'Mission complete — {len(self.spawned)} markers left on field')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DiabloVisuals()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Ctrl+C')
    except Exception as e:
        print(f'diablo_visuals error: {e}', file=sys.stderr)
        raise
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
