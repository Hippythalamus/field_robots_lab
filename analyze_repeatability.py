#!/usr/bin/env python3
"""
Repeatability analysis for the tank-farm-patrol 15-run calibration baseline.

Reads every tank_farm_run_NN bag under ~/field_robots_lab_experiments/,
extracts:
  * the two scenario barriers (/scenario_runner/start, /complete)
  * per-robot mission_status timestamps
  * /robot_N/inspection_event payloads (decisions, mission_time_s, pose)
  * publish-rate stats for high-rate sensor topics (imu, odom, joint_states)

and writes a Markdown report summarising cross-run statistics. The
report is meant to be the evidence table referenced from the Mission
Briefing document.

Usage:
  ./analyze_repeatability.py
  ./analyze_repeatability.py --runs-dir /custom/path
  ./analyze_repeatability.py --out /custom/report.md

Dependencies: rosbag2_py + rclpy (already on the system from ROS 2
Humble). No third-party Python packages required.
"""

import argparse
import json
import os
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROBOT_NAMESPACES = ['robot_1', 'robot_2', 'robot_3']

RATE_TOPICS: Dict[str, float] = {
    '/robot_1/imu/data_raw': 83.0,
    '/robot_2/imu/data_raw': 83.0,
    '/robot_3/imu/data_raw': 83.0,
    '/robot_1/odom':         38.0,
    '/robot_2/odom':         38.0,
    '/robot_3/odom':         38.0,
    '/robot_1/joint_states': 47.0,
    '/robot_2/joint_states': 47.0,
    '/robot_3/joint_states': 47.0,
}


@dataclass
class RunMetrics:
    run_id: str
    bag_path: Path
    start_t: Optional[float] = None
    complete_t: Optional[float] = None
    status_t: Dict[str, float] = field(default_factory=dict)
    inspections: List[dict] = field(default_factory=list)
    rates: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id='mcap',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def topic_type_map(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def analyse_bag(run_id: str, bag_dir: Path) -> RunMetrics:
    m = RunMetrics(run_id=run_id, bag_path=bag_dir)

    if not (bag_dir / 'metadata.yaml').exists():
        m.notes.append('missing metadata.yaml')
        return m

    reader = open_reader(bag_dir)
    type_by_topic = topic_type_map(reader)

    # First pass: collect inspection events, barriers, status timestamps,
    # and timestamps per rate topic for rate analysis.
    rate_timestamps: Dict[str, List[float]] = defaultdict(list)

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9

        if topic == '/scenario_runner/start' and m.start_t is None:
            m.start_t = t
        elif topic == '/scenario_runner/complete' and m.complete_t is None:
            m.complete_t = t
        elif topic.startswith('/robot_') and topic.endswith('/mission_status'):
            ns = topic.split('/')[1]
            if ns not in m.status_t:
                m.status_t[ns] = t
        elif topic.startswith('/robot_') and topic.endswith('/inspection_event'):
            try:
                msg_type = get_message(type_by_topic[topic])
                msg = deserialize_message(raw, msg_type)
                event = json.loads(msg.data)
                if event.get('event') == 'inspection_completed':
                    event['_recv_t'] = t
                    event['_robot'] = topic.split('/')[1]
                    m.inspections.append(event)
            except Exception as e:
                m.notes.append(f'bad inspection event on {topic}: {e}')
        elif topic in RATE_TOPICS:
            rate_timestamps[topic].append(t)

    # Rate stats: derive from inter-message intervals between start and complete
    for topic, ts in rate_timestamps.items():
        if len(ts) < 2:
            m.rates[topic] = (0.0, 0.0)
            continue
        if m.start_t and m.complete_t:
            ts = [t for t in ts if m.start_t <= t <= m.complete_t]
            if len(ts) < 2:
                m.rates[topic] = (0.0, 0.0)
                continue
        deltas = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        if not deltas:
            m.rates[topic] = (0.0, 0.0)
            continue
        mean_dt = statistics.mean(deltas)
        rate = 1.0 / mean_dt if mean_dt > 0 else 0.0
        std_dt = statistics.stdev(deltas) if len(deltas) > 1 else 0.0
        m.rates[topic] = (rate, std_dt / mean_dt if mean_dt > 0 else 0.0)

    return m


def summarise(values: List[float]) -> dict:
    if not values:
        return {'n': 0, 'mean': None, 'std': None,
                'min': None, 'max': None, 'cv_pct': None}
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    cv = (std / mean * 100.0) if mean else 0.0
    return {
        'n': len(values),
        'mean': mean,
        'std': std,
        'min': min(values),
        'max': max(values),
        'cv_pct': cv,
    }


def fmt(v, spec='.3f'):
    if v is None:
        return 'n/a'
    return format(v, spec)


def render_report(runs: List[RunMetrics], out_path: Path):
    lines: List[str] = []
    p = lines.append

    p('# Tank Farm Patrol — Repeatability Report')
    p('')
    p('Cross-run statistics for the 15-run calibration baseline.')
    p(f'Source: {runs[0].bag_path.parent.parent if runs else "n/a"}')
    p(f'Runs analysed: {len(runs)}')
    p('')

    # === 1. Mission duration ===
    p('## 1. Mission Duration (barrier-to-barrier)')
    p('')
    p('Time between `/scenario_runner/start` (all robots ready) and '
      '`/scenario_runner/complete` (all robots finished). This is the '
      'deterministic mission window, independent of spawn-order drift.')
    p('')
    durations = [
        r.complete_t - r.start_t
        for r in runs
        if r.start_t is not None and r.complete_t is not None
    ]
    s = summarise(durations)
    p('| metric | value |')
    p('|---|---|')
    p(f'| Runs with both barriers | {s["n"]} / {len(runs)} |')
    p(f'| Mean (s) | {fmt(s["mean"])} |')
    p(f'| Std (s) | {fmt(s["std"])} |')
    p(f'| Min (s) | {fmt(s["min"])} |')
    p(f'| Max (s) | {fmt(s["max"])} |')
    p(f'| CV (%) | {fmt(s["cv_pct"], ".2f")} |')
    p('')
    p('### Per-run mission duration')
    p('')
    p('| run | start_t | complete_t | duration_s |')
    p('|---|---|---|---|')
    for r in runs:
        if r.start_t is not None and r.complete_t is not None:
            p(f'| {r.run_id} | {r.start_t:.3f} | {r.complete_t:.3f} | '
              f'{r.complete_t - r.start_t:.3f} |')
        else:
            p(f'| {r.run_id} | (missing) | (missing) | n/a |')
    p('')

    # === 2. Per-robot mission duration ===
    p('## 2. Per-Robot Mission Duration')
    p('')
    p('Time between `/scenario_runner/start` and each robot\'s '
      '`mission_status=True`. Per-robot variance is the patrol-loop '
      'consistency for that namespace.')
    p('')
    p('| robot | n | mean_s | std_s | cv_% | min_s | max_s |')
    p('|---|---|---|---|---|---|---|')
    for ns in ROBOT_NAMESPACES:
        per_robot = [
            r.status_t[ns] - r.start_t
            for r in runs
            if r.start_t is not None and ns in r.status_t
        ]
        s = summarise(per_robot)
        p(f'| {ns} | {s["n"]} | {fmt(s["mean"])} | {fmt(s["std"])} | '
          f'{fmt(s["cv_pct"], ".2f")} | {fmt(s["min"])} | {fmt(s["max"])} |')
    p('')

    # === 3. Inter-robot completion spread ===
    p('## 3. Inter-Robot Completion Spread')
    p('')
    p('Range between earliest and latest `mission_status=True` within '
      'one run. Indicates how tightly the three navigators finish '
      'their respective patrol assignments. Wider spread is by design: '
      'robot_1 performs the deep patrol (~95 m) while robot_2/robot_3 '
      'cover near-side sectors (~50-60 m each).')
    p('')
    spreads = []
    for r in runs:
        ts = list(r.status_t.values())
        if len(ts) == 3:
            spreads.append(max(ts) - min(ts))
    s = summarise(spreads)
    p('| metric | value |')
    p('|---|---|')
    p(f'| Runs analysed | {s["n"]} / {len(runs)} |')
    p(f'| Mean spread (s) | {fmt(s["mean"])} |')
    p(f'| Std spread (s) | {fmt(s["std"])} |')
    p(f'| Min spread (s) | {fmt(s["min"])} |')
    p(f'| Max spread (s) | {fmt(s["max"])} |')
    p('')

    # === 4. Inspection decisions ===
    p('## 4. Inspection Decisions')
    p('')
    p('Each tank is inspected exactly once per run. Pressure and methane '
      'readings are deterministic (read from scenario YAML), so decisions '
      'must be identical across all 15 runs. Mission-time of each '
      'inspection varies — that is the physics-noise signature.')
    p('')

    by_tank: Dict[str, List[dict]] = defaultdict(list)
    for r in runs:
        for ev in r.inspections:
            by_tank[ev.get('tank_id', '?')].append(ev)

    p('| tank | n_runs | decision | pressure_bar | methane_ppm | '
      'mission_time mean | mission_time std | cv_% |')
    p('|---|---|---|---|---|---|---|---|')
    for tank_id in sorted(by_tank.keys()):
        evs = by_tank[tank_id]
        decisions = set(ev.get('decision') for ev in evs)
        pressures = set(round(ev.get('pressure_bar', 0.0), 3) for ev in evs)
        methanes = set(round(ev.get('methane_ppm', 0.0), 3) for ev in evs)
        times = [ev.get('mission_time_s') for ev in evs
                 if ev.get('mission_time_s') is not None]
        s = summarise(times)
        decision_str = ','.join(sorted(decisions))
        pressure_str = ','.join(str(p) for p in sorted(pressures))
        methane_str = ','.join(str(m) for m in sorted(methanes))
        p(f'| {tank_id} | {len(evs)} | {decision_str} | {pressure_str} | '
          f'{methane_str} | {fmt(s["mean"])} | {fmt(s["std"])} | '
          f'{fmt(s["cv_pct"], ".2f")} |')
    p('')
    # Determinism check
    nondeterministic = [
        tank_id for tank_id, evs in by_tank.items()
        if len(set(ev.get('decision') for ev in evs)) > 1
    ]
    if nondeterministic:
        p('**WARNING:** decision varied across runs for tanks: '
          f'{", ".join(nondeterministic)} — investigate.')
    else:
        p('**Decision determinism: PASS** — every tank has a single '
          'decision across all runs.')
    p('')

    # === 5. Telemetry rates ===
    p('## 5. Telemetry Publish Rates')
    p('')
    p('Mean publish rate per topic, measured between the start and '
      'complete barriers. Cross-run CV indicates rate stability — the '
      'kind of signature an integrity-monitoring layer (RISQ) would '
      'baseline against.')
    p('')
    p('| topic | expected_hz | mean_hz | cross-run std | cross-run cv_% |')
    p('|---|---|---|---|---|')
    for topic, expected in RATE_TOPICS.items():
        rates = [r.rates[topic][0] for r in runs if topic in r.rates
                 and r.rates[topic][0] > 0]
        s = summarise(rates)
        p(f'| {topic} | {expected:.0f} | {fmt(s["mean"], ".2f")} | '
          f'{fmt(s["std"], ".3f")} | {fmt(s["cv_pct"], ".2f")} |')
    p('')

    # === 6. Notes ===
    issues = [(r.run_id, r.notes) for r in runs if r.notes]
    if issues:
        p('## 6. Per-Run Notes')
        p('')
        for run_id, notes in issues:
            p(f'- **{run_id}**: {"; ".join(notes)}')
        p('')

    p('---')
    p('Generated by `analyze_repeatability.py`.')

    out_path.write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--runs-dir', type=Path,
        default=Path.home() / 'field_robots_lab_experiments',
        help='Parent directory containing tank_farm_run_NN/ subdirs',
    )
    parser.add_argument(
        '--out', type=Path,
        default=Path.home() / 'field_robots_lab_experiments' /
                'repeatability_report.md',
        help='Path to write the Markdown report',
    )
    parser.add_argument(
        '--prefix', default='tank_farm_run_',
        help='Run directory prefix to match',
    )
    args = parser.parse_args()

    run_dirs = sorted(
        d for d in args.runs_dir.iterdir()
        if d.is_dir() and d.name.startswith(args.prefix)
    )
    if not run_dirs:
        print(f'No run dirs matching {args.prefix}* in {args.runs_dir}',
              file=sys.stderr)
        sys.exit(1)

    print(f'Analysing {len(run_dirs)} runs from {args.runs_dir} ...')
    runs: List[RunMetrics] = []
    for d in run_dirs:
        run_id = d.name.replace(args.prefix, '')
        bag_dir = d / 'bag'
        if not bag_dir.is_dir():
            print(f'  {run_id}: no bag/ subdir, skipping')
            continue
        try:
            m = analyse_bag(run_id, bag_dir)
        except Exception as e:
            print(f'  {run_id}: error — {e}')
            continue
        runs.append(m)
        dur = (m.complete_t - m.start_t
               if m.start_t and m.complete_t else None)
        print(f'  {run_id}: duration={fmt(dur)}s, inspections={len(m.inspections)}')

    if not runs:
        print('No runs successfully analysed.', file=sys.stderr)
        sys.exit(2)

    render_report(runs, args.out)
    print(f'\nReport written to: {args.out}')


if __name__ == '__main__':
    main()
