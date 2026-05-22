"""
Telemetry recorder node.

Spawns 'ros2 bag record' as a subprocess and writes experiment metadata
alongside the bag. Does NOT subscribe to data topics itself — recording
is delegated to native rosbag2 to avoid Python overhead on high-rate
streams.

Lifecycle:
1. Read topics from YAML config
2. Create experiment output directory
3. Write metadata.yaml with experiment info
4. Launch 'ros2 bag record' subprocess
5. Run until SIGINT (Ctrl+C)
6. Terminate subprocess cleanly, write end timestamp to metadata
"""

import os
import sys
import signal
import subprocess
import datetime
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node


class RecorderNode(Node):
    def __init__(self):
        super().__init__('telemetry_recorder')

        # Parameters
        self.declare_parameter('topics_config', '')
        self.declare_parameter('output_dir', '/tmp/field_robots_lab/experiments')
        self.declare_parameter('experiment_name', '')

        self.topics_config_path = self.get_parameter('topics_config').value
        self.output_dir = self.get_parameter('output_dir').value
        self.experiment_name = self.get_parameter('experiment_name').value

        if not self.topics_config_path or not os.path.isfile(self.topics_config_path):
            self.get_logger().error(
                f'topics_config not found: {self.topics_config_path}')
            raise FileNotFoundError(self.topics_config_path)

        # Load config
        with open(self.topics_config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Resolve experiment name
        if not self.experiment_name:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            scene = self.config['experiment']['scene']
            self.experiment_name = f'{scene}_{ts}'

        self.exp_dir = Path(self.output_dir) / self.experiment_name
        self.exp_dir.mkdir(parents=True, exist_ok=True)

        # Persist metadata
        self.metadata = {
            'experiment_name': self.experiment_name,
            'scene': self.config['experiment']['scene'],
            'robot': self.config['experiment']['robot'],
            'started_at': datetime.datetime.now().isoformat(),
            'topics': [t['name'] for t in self.config['topics']],
            'storage_format': self.config['storage']['format'],
        }
        self._write_metadata()

        # Launch ros2 bag record subprocess
        topic_names = [t['name'] for t in self.config['topics']]
        bag_path = str(self.exp_dir / 'bag')

        cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '-s', self.config['storage']['format'],
        ]
        compression = self.config['storage'].get('compression')
        if compression:
            cmd += ['--compression-mode', 'file',
                    '--compression-format', compression]
        cmd += topic_names

        self.get_logger().info(f'Starting recording to: {bag_path}')
        self.get_logger().info(f'Topics: {topic_names}')

        self.bag_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        self.get_logger().info(
            f'Recording started. Experiment dir: {self.exp_dir}')
        self.get_logger().info('Press Ctrl+C to stop.')

    def _write_metadata(self):
        meta_path = self.exp_dir / 'metadata.yaml'
        with open(meta_path, 'w') as f:
            yaml.safe_dump(self.metadata, f, default_flow_style=False)

    def shutdown(self):
        """Cleanly stop ros2 bag record and finalize metadata."""
        if hasattr(self, 'bag_proc') and self.bag_proc.poll() is None:
            self.get_logger().info('Stopping ros2 bag record...')
            # Send SIGINT to the process group so rosbag flushes properly
            os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGINT)
            try:
                self.bag_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('rosbag did not exit in 10s, killing.')
                os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGKILL)
                self.bag_proc.wait()

        self.metadata['stopped_at'] = datetime.datetime.now().isoformat()
        self._write_metadata()
        self.get_logger().info(f'Recording finalized: {self.exp_dir}')




def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Ctrl+C received.')
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'Recorder failed: {e}')
        raise
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception as e:
                print(f'Warning: error during shutdown: {e}', file=sys.stderr)
            node.destroy_node()
        # Only call shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
