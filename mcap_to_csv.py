#!/usr/bin/env python3
"""
Export a single mission bag to a RideScan-compatible CSV.

Produces the WHEELED_MOBILE schema with 9 full features:
  - pose_position_x/y/z          (from /<ns>/odom)
  - pose_orientation_roll/pitch/yaw  (quaternion -> Euler)
  - twist_linear_x/y/z           (from /<ns>/odom)
  - twist_angular_x/y/z          (from /<ns>/odom)
  - linear_acceleration_x/y/z    (from /<ns>/imu/data_raw, interpolated)
  - gps_lat / gps_lon / gps_alt  (from /<ns>/gps/data, interpolated)
  - timestamp                    (sim-time seconds, relative to mission start)

Time domain handling
--------------------
The bag has TWO distinct time domains:
  * wall time — when the recorder received each message (rosbag2 internal)
  * sim time  — Gazebo's simulation clock, carried in each message's
                header.stamp

The two diverge under CPU load: Gazebo runs slower than real time, so
1 s of wall might contain only 0.7 s of sim. Sensor data physics
(velocity, acceleration) is consistent with sim time, not wall time —
so the CSV must use sim time for timestamps.

The /scenario_runner/start and /complete barriers, however, are
emitted in wall time (rosbag receive time). To compute the mission
window in sim time we use the /clock topic, which publishes the
sim-clock value as a wall-time message — giving us a sequence of
(wall_t, sim_t) pairs that we use to interpolate the barriers from
wall to sim time.

Validation per RideScan spec:
  - All columns convert to float
  - No NaN, None or empty cells
  - Monotonically increasing timestamps
  - Length >= 16 samples
  - Euler angles in [-pi, pi]
  - Positions finite
"""

import argparse
import csv
import math
import sys
from bisect import bisect_left
from pathlib import Path
from typing import List, Optional, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


COLUMNS = [
    'timestamp',
    'pose_position_x', 'pose_position_y', 'pose_position_z',
    'pose_orientation_roll', 'pose_orientation_pitch', 'pose_orientation_yaw',
    'twist_linear_x', 'twist_linear_y', 'twist_linear_z',
    'twist_angular_x', 'twist_angular_y', 'twist_angular_z',
    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
    'gps_lat', 'gps_lon', 'gps_alt',
]


def quaternion_to_euler(x: float, y: float, z: float, w: float
                        ) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def open_reader(bag_dir: Path) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir), storage_id='mcap',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def interp(ts_query: float,
           xs: List[float], ys: List[float]) -> Optional[float]:
    """Linear interpolation. Returns None if query is outside xs range."""
    if not xs or ts_query < xs[0] or ts_query > xs[-1]:
        return None
    i = bisect_left(xs, ts_query)
    if i == 0:
        return ys[0]
    if i >= len(xs):
        return ys[-1]
    if xs[i] == ts_query:
        return ys[i]
    x0, x1 = xs[i - 1], xs[i]
    y0, y1 = ys[i - 1], ys[i]
    if x1 == x0:
        return y0
    alpha = (ts_query - x0) / (x1 - x0)
    return y0 + alpha * (y1 - y0)


def export(bag_dir: Path, robot_ns: str, out_path: Path) -> int:
    reader = open_reader(bag_dir)
    type_by_topic = {t.name: t.type
                     for t in reader.get_all_topics_and_types()}

    odom_topic = f'/{robot_ns}/odom'
    imu_topic = f'/{robot_ns}/imu/data_raw'
    gps_topic = f'/{robot_ns}/gps/data'

    for required in (odom_topic, imu_topic, gps_topic, '/clock'):
        if required not in type_by_topic:
            raise RuntimeError(f'Topic {required} missing from {bag_dir}')

    # Wall-time barriers
    barrier_start_wall: Optional[float] = None
    barrier_complete_wall: Optional[float] = None

    # /clock pairs — recv_wall_t -> sim_t
    clock_wall: List[float] = []
    clock_sim: List[float] = []

    # Sensor streams in SIM time (from header.stamp)
    odom_rows: List[dict] = []
    imu_ts: List[float] = []
    imu_ax: List[float] = []
    imu_ay: List[float] = []
    imu_az: List[float] = []
    gps_ts: List[float] = []
    gps_lat: List[float] = []
    gps_lon: List[float] = []
    gps_alt: List[float] = []

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        recv_wall = t_ns * 1e-9

        if topic == '/scenario_runner/start':
            if barrier_start_wall is None:
                barrier_start_wall = recv_wall
        elif topic == '/scenario_runner/complete':
            if barrier_complete_wall is None:
                barrier_complete_wall = recv_wall
        elif topic == '/clock':
            msg_type = get_message(type_by_topic[topic])
            msg = deserialize_message(raw, msg_type)
            clock_wall.append(recv_wall)
            clock_sim.append(stamp_to_seconds(msg.clock))
        elif topic == odom_topic:
            msg_type = get_message(type_by_topic[topic])
            msg = deserialize_message(raw, msg_type)
            t = stamp_to_seconds(msg.header.stamp)
            ox = msg.pose.pose.orientation.x
            oy = msg.pose.pose.orientation.y
            oz = msg.pose.pose.orientation.z
            ow = msg.pose.pose.orientation.w
            roll, pitch, yaw = quaternion_to_euler(ox, oy, oz, ow)
            odom_rows.append({
                't': t,
                'px': msg.pose.pose.position.x,
                'py': msg.pose.pose.position.y,
                'pz': msg.pose.pose.position.z,
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'lx': msg.twist.twist.linear.x,
                'ly': msg.twist.twist.linear.y,
                'lz': msg.twist.twist.linear.z,
                'ax': msg.twist.twist.angular.x,
                'ay': msg.twist.twist.angular.y,
                'az': msg.twist.twist.angular.z,
            })
        elif topic == imu_topic:
            msg_type = get_message(type_by_topic[topic])
            msg = deserialize_message(raw, msg_type)
            t = stamp_to_seconds(msg.header.stamp)
            imu_ts.append(t)
            imu_ax.append(msg.linear_acceleration.x)
            imu_ay.append(msg.linear_acceleration.y)
            imu_az.append(msg.linear_acceleration.z)
        elif topic == gps_topic:
            msg_type = get_message(type_by_topic[topic])
            msg = deserialize_message(raw, msg_type)
            t = stamp_to_seconds(msg.header.stamp)
            gps_ts.append(t)
            gps_lat.append(msg.latitude)
            gps_lon.append(msg.longitude)
            gps_alt.append(msg.altitude)

    if barrier_start_wall is None or barrier_complete_wall is None:
        raise RuntimeError(
            f'{bag_dir}: missing /scenario_runner/start or /complete barrier')
    if not clock_wall:
        raise RuntimeError(f'{bag_dir}: no /clock messages — cannot map time')

    # Sort streams (rosbag2 generally yields in order, but be safe)
    odom_rows.sort(key=lambda r: r['t'])

    def sort_stream(ts, *vals):
        order = sorted(range(len(ts)), key=lambda i: ts[i])
        ts_sorted = [ts[i] for i in order]
        vals_sorted = [[v[i] for i in order] for v in vals]
        return (ts_sorted, *vals_sorted)

    imu_ts, imu_ax, imu_ay, imu_az = sort_stream(imu_ts, imu_ax, imu_ay, imu_az)
    gps_ts, gps_lat, gps_lon, gps_alt = sort_stream(
        gps_ts, gps_lat, gps_lon, gps_alt)
    clock_pairs_sorted = sorted(zip(clock_wall, clock_sim))
    clock_wall = [w for w, _ in clock_pairs_sorted]
    clock_sim = [s for _, s in clock_pairs_sorted]

    # Map barrier wall times -> sim times
    start_sim = interp(barrier_start_wall, clock_wall, clock_sim)
    complete_sim = interp(barrier_complete_wall, clock_wall, clock_sim)
    if start_sim is None or complete_sim is None:
        raise RuntimeError(
            f'{bag_dir}: barrier wall times outside /clock range '
            f'(start_wall={barrier_start_wall}, complete_wall={barrier_complete_wall}, '
            f'clock_wall=[{clock_wall[0]}, {clock_wall[-1]}])')

    if complete_sim <= start_sim:
        raise RuntimeError(
            f'{bag_dir}: complete_sim {complete_sim} <= start_sim {start_sim}')

    # Filter odom rows to mission window [start_sim, complete_sim].
    # Also require IMU + GPS coverage at each kept timestamp so interp is safe.
    imu_lo, imu_hi = (imu_ts[0], imu_ts[-1]) if imu_ts else (None, None)
    gps_lo, gps_hi = (gps_ts[0], gps_ts[-1]) if gps_ts else (None, None)

    kept: List[dict] = []
    for r in odom_rows:
        t = r['t']
        if t < start_sim or t > complete_sim:
            continue
        if imu_lo is None or t < imu_lo or t > imu_hi:
            continue
        if gps_lo is None or t < gps_lo or t > gps_hi:
            continue
        kept.append(r)

    if len(kept) < 16:
        raise RuntimeError(
            f'{bag_dir}: only {len(kept)} odom samples in valid sim-time '
            f'window [{start_sim:.3f}, {complete_sim:.3f}] (need >= 16). '
            f'odom sim range: [{odom_rows[0]["t"]:.3f}, {odom_rows[-1]["t"]:.3f}], '
            f'imu sim range: [{imu_lo}, {imu_hi}], '
            f'gps sim range: [{gps_lo}, {gps_hi}]')

    # Write CSV
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)

        for r in kept:
            t_rel = r['t'] - start_sim

            ax = interp(r['t'], imu_ts, imu_ax)
            ay = interp(r['t'], imu_ts, imu_ay)
            az = interp(r['t'], imu_ts, imu_az)
            lat = interp(r['t'], gps_ts, gps_lat)
            lon = interp(r['t'], gps_ts, gps_lon)
            alt = interp(r['t'], gps_ts, gps_alt)

            row = [
                t_rel,
                r['px'], r['py'], r['pz'],
                r['roll'], r['pitch'], r['yaw'],
                r['lx'], r['ly'], r['lz'],
                r['ax'], r['ay'], r['az'],
                ax, ay, az,
                lat, lon, alt,
            ]
            for i, v in enumerate(row):
                if v is None or not math.isfinite(v):
                    raise RuntimeError(
                        f'{bag_dir}: non-finite or missing value '
                        f'in column {COLUMNS[i]} at t={t_rel:.3f}')
            writer.writerow([f'{v:.9g}' for v in row])

    return len(kept)


def validate(csv_path: Path) -> dict:
    with csv_path.open() as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    issues: List[str] = []
    if not rows:
        issues.append('empty file')
        return {'rows': 0, 'issues': issues}

    if len(rows) < 16:
        issues.append(f'fewer than 16 samples ({len(rows)})')

    headers = list(rows[0].keys())
    missing = [c for c in COLUMNS if c not in headers]
    if missing:
        issues.append(f'missing columns: {missing}')

    last_t = None
    for i, row in enumerate(rows):
        for col in COLUMNS:
            v = row.get(col, '')
            if v == '' or v is None:
                issues.append(f'row {i}: empty value in {col}')
                continue
            try:
                fv = float(v)
            except ValueError:
                issues.append(f'row {i}: non-numeric in {col} = {v!r}')
                continue
            if math.isnan(fv) or math.isinf(fv):
                issues.append(f'row {i}: NaN/Inf in {col}')

        try:
            t = float(row['timestamp'])
            if last_t is not None and t < last_t:
                issues.append(f'row {i}: timestamp not monotonic '
                              f'(prev {last_t}, this {t})')
            last_t = t
        except Exception:
            pass

        for col in ('pose_orientation_roll',
                    'pose_orientation_pitch',
                    'pose_orientation_yaw'):
            try:
                v = float(row[col])
                if v < -math.pi - 1e-6 or v > math.pi + 1e-6:
                    issues.append(f'row {i}: {col}={v} outside [-pi, pi]')
            except Exception:
                pass

    return {'rows': len(rows), 'issues': issues}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', type=Path, required=True,
                   help='Path to bag directory containing *.mcap')
    p.add_argument('--robot', required=True,
                   help='Robot namespace (e.g. robot_1)')
    p.add_argument('--out', type=Path, required=True,
                   help='Output CSV path')
    p.add_argument('--no-validate', action='store_true',
                   help='Skip post-write validation')
    args = p.parse_args()

    try:
        n_rows = export(args.bag, args.robot, args.out)
    except Exception as e:
        print(f'EXPORT FAILED: {e}', file=sys.stderr)
        sys.exit(2)

    print(f'wrote {n_rows} rows to {args.out}')

    if not args.no_validate:
        result = validate(args.out)
        if result['issues']:
            print(f'VALIDATION ISSUES ({len(result["issues"])}):',
                  file=sys.stderr)
            for issue in result['issues'][:20]:
                print(f'  - {issue}', file=sys.stderr)
            if len(result['issues']) > 20:
                print(f'  ... and {len(result["issues"]) - 20} more',
                      file=sys.stderr)
            sys.exit(3)
        else:
            print(f'validation: PASS ({result["rows"]} rows)')


if __name__ == '__main__':
    main()
