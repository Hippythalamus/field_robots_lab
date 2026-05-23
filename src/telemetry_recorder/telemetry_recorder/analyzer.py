"""
Post-hoc bag analyzer.

Usage:
    ros2 run telemetry_recorder analyzer <experiment_dir>

Reads the rosbag in <experiment_dir>/bag/, computes per-topic metrics,
writes <experiment_dir>/metrics.json, and prints a summary table.
"""

import argparse
import json
import os
import sys
import yaml
from pathlib import Path
from collections import defaultdict
from datetime import datetime

from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions,
)

from telemetry_recorder.metrics import compute_topic_metrics


def find_bag_path(exp_dir: Path) -> Path:
    """Locate the bag inside the experiment directory."""
    bag_dir = exp_dir / 'bag'
    if not bag_dir.is_dir():
        raise FileNotFoundError(f'Bag directory not found: {bag_dir}')
    return bag_dir


def load_expected_rates(exp_dir: Path) -> dict:
    """
    Load expected_rate_hz per topic from topics_config.yaml.
    """
    config_path = exp_dir / 'topics_config.yaml'
    if not config_path.is_file():
        return {}
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    rates = {}
    for topic_entry in config.get('topics', []):
        name = topic_entry.get('name')
        rate = topic_entry.get('expected_rate_hz')
        if name and rate is not None:
            rates[name] = rate
    return rates


def collect_timestamps(bag_dir: Path) -> dict:
    """
    Open the bag and collect receive timestamps per topic.

    Returns
    -------
    dict
        {topic_name: [t_ns, t_ns, ...]}
    """
    storage_options = StorageOptions(uri=str(bag_dir), storage_id='mcap')
    converter_options = ConverterOptions('', '')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics_by_name = {}
    for topic_metadata in reader.get_all_topics_and_types():
        topics_by_name[topic_metadata.name] = topic_metadata.type

    timestamps = defaultdict(list)
    while reader.has_next():
        topic, _data, t_ns = reader.read_next()
        timestamps[topic].append(t_ns)

    return dict(timestamps), topics_by_name


def analyze_experiment(exp_dir: Path) -> dict:
    """Run full analysis on one experiment directory."""
    bag_dir = find_bag_path(exp_dir)
    expected_rates = load_expected_rates(exp_dir)

    print(f'Reading bag: {bag_dir}')
    timestamps, topics_by_name = collect_timestamps(bag_dir)

    per_topic = {}
    for topic, ts_list in sorted(timestamps.items()):
        expected = expected_rates.get(topic)
        per_topic[topic] = {
            'type': topics_by_name.get(topic, 'unknown'),
            'metrics': compute_topic_metrics(ts_list, expected_rate_hz=expected),
        }

    # Overall summary
    all_msg_count = sum(len(ts) for ts in timestamps.values())
    summary = {
        'analyzed_at': datetime.now().isoformat(),
        'experiment_dir': str(exp_dir),
        'topic_count': len(timestamps),
        'total_message_count': all_msg_count,
        'topics': per_topic,
    }

    return summary


def print_summary_table(summary: dict) -> None:
    """Print a human-readable summary to stdout."""
    print()
    print(f"{'Topic':<30} {'Count':>8} {'Rate Hz':>10} {'p95 ms':>10} {'p99 ms':>10} {'Gaps':>6}")
    print('-' * 80)
    for topic, info in summary['topics'].items():
        m = info['metrics']
        if m.get('message_count', 0) < 2:
            print(f"{topic:<30} {m.get('message_count', 0):>8} {'-':>10} {'-':>10} {'-':>10} {'-':>6}")
            continue
        rate = m['actual_rate_hz']
        p95_ms = m['inter_arrival_time_s']['p95'] * 1000
        p99_ms = m['inter_arrival_time_s']['p99'] * 1000
        gaps = m['gap_count']
        print(
            f"{topic:<30} {m['message_count']:>8} {rate:>10.2f} {p95_ms:>10.2f} {p99_ms:>10.2f} {gaps:>6}"
        )
    print()


def main():
    parser = argparse.ArgumentParser(description='Post-hoc bag analyzer.')
    parser.add_argument(
        'experiment_dir',
        type=str,
        help='Path to experiment directory containing bag/ subfolder.',
    )
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Where to write metrics.json. Default: <experiment_dir>/metrics.json',
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppress summary table.',
    )
    args = parser.parse_args()

    exp_dir = Path(args.experiment_dir).expanduser().resolve()
    if not exp_dir.is_dir():
        print(f'Error: not a directory: {exp_dir}', file=sys.stderr)
        sys.exit(1)

    summary = analyze_experiment(exp_dir)

    output_path = Path(args.output) if args.output else exp_dir / 'metrics.json'
    with open(output_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'Metrics written to: {output_path}')

    if not args.quiet:
        print_summary_table(summary)


if __name__ == '__main__':
    main()
