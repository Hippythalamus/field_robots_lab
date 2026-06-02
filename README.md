# field_robots_lab

Reproducible Gazebo scenes for field robotics: mobile platforms, UAVs,
quadrupeds, and small heterogeneous fleets. Focus on **realistic
telemetry under varying load**, not autonomy stacks or polished demos.

## Why this exists

Most public ROS2 / Gazebo material targets one of two ends: introductory
tutorials with a single TurtleBot in an empty world, or full autonomy
stacks (Nav2, MoveIt) on standardized platforms. Neither is the right
fit for studying *system-level reliability* — how telemetry, transport,
and compute behave when a real fleet runs under realistic conditions.

`field_robots_lab` fills the gap with a small set of scenes built around
platforms that are actually deployed in field robotics (skid-steer AGVs,
industrial UAVs, quadruped inspection robots), instrumented for
controlled load experiments rather than headline-grabbing autonomy.

## Scenes

| # | Robot | Scenario | Status |
|---|-------|----------|--------|
| 1 | Scout Mini (AgileX) | warehouse navigation baseline | working — see Quick Start |
| 2 | Heterogeneous fleet | three Scout Minis with mixed sensor configs | working — see Quick Start |
| 3 | DJI M350-class surrogate (PX4 SITL) | pipeline inspection | planned |
| 4 | Quadruped (Go2 / ANYmal-class) | indoor inspection | planned |

## What this is not

- Not a tutorial repo — assumes ROS2 working knowledge
- Not a Nav2/MoveIt showcase — scenes are deliberately minimal in autonomy
- Not a benchmark suite — scenes are research stands, not standardized tests
- Not production-ready — research prototype, see Known limitations

## Stack

- ROS2 Humble
- Gazebo Classic 11
- Ubuntu 22.04

Other simulators may be added where they fit better (e.g., MuJoCo for
quadruped contact dynamics) — noted per scene.

## Repository structure

## Repository structure
field_robots_lab/
├── src/
│   ├── scout_mini_description/   # URDF, Gazebo plugins, world, launch
│   └── telemetry_recorder/        # Recording wrapper + analyzer
│   └── scenario_runner/           # Repeatable scenarios with PD navigation
├── docs/                          # Per-scene documentation (planned)
└── README.md

Sensors are isolated into parameterized xacro macros
(`urdf/sensors/imu.xacro`, `urdf/sensors/lidar_2d.xacro`), so each
robot instance can be configured with different sensor profiles
without touching the base URDF. The main robot xacro accepts
`use_imu`, `use_lidar`, `lidar_samples`, `imu_rate`, and `namespace`
as launch-supplied arguments.

Scene-specific files (worlds, launch files) currently live inside the
robot description package. They will be reorganized into a top-level
`scenes/` directory in a future revision.

## Quick Start

Requires Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11 + `gazebo_ros_pkgs`.

### Build

```bash
git clone https://github.com/Hippythalamus/field_robots_lab.git
cd field_robots_lab
colcon build --symlink-install
source install/setup.bash
```

### Run Scene 1: Scout Mini in warehouse

```bash
ros2 launch scout_mini_description gazebo.launch.py
```

One Scout Mini spawns in a 20×20 m enclosed warehouse with six static
obstacles. Wait ~30 seconds for Gazebo to load. The robot publishes:

- `/imu` at 100 Hz
- `/scan` (2D lidar) at 10 Hz, 360°, 0.12–12 m range
- `/odom` at 50 Hz from skid-steer plugin
- `/joint_states`, `/tf`, `/tf_static`

### Run Scene 2: Heterogeneous fleet

```bash
ros2 launch scout_mini_description fleet.launch.py
```

Three Scout Minis share one Gazebo simulation, each in its own ROS2
namespace:

- `robot_0` — full sensor suite (IMU 100 Hz + Lidar 360 samples)
- `robot_1` — reduced lidar resolution (IMU 100 Hz + Lidar 180 samples)
- `robot_2` — IMU only, no lidar

Telemetry is published under per-robot namespaces:
`/robot_0/imu`, `/robot_0/scan`, `/robot_0/odom`, `/robot_0/cmd_vel`,
and equivalents for `robot_1` and `robot_2` (no `/robot_2/scan`).

### Run a repeatable scenario (with automated PD navigation)

```bash
ros2 launch scenario_runner scenario_navigation.launch.py \
  experiment_name:=my_run_01
```

This is the end-to-end repeatable run for Scene 2:

- Gazebo + warehouse world
- 3 Scout Minis with configured sensor profiles
- 3 `waypoint_navigator` nodes — one per robot, each with its own
  waypoint list defined in `scenario_runner/config/fleet_scenario.yaml`
- Telemetry recorder writing to
  `~/field_robots_lab_experiments/my_run_01/`

Each navigator drives its robot through a small rectangular path
using a PD controller. No teleoperation. When all navigators finish,
the run can be stopped with `Ctrl+C`.

A typical run takes 60–90 seconds and produces a complete
experiment directory with bag, metadata, and topics config.

### Manual driving (Scenes 1 and 2 without scenarios)

```bash
# Scene 1
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.2}}"

# Scene 2: target a specific robot
ros2 topic pub --rate 10 /robot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.1}}"
```

Or keyboard teleop:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Record telemetry separately (without scenario_runner)

The recorder can be used standalone with any of the launches above.
Scene 1 uses `scout_mini_topics.yaml` (default); Scene 2 uses
`fleet_topics.yaml`:

```bash
# Scene 1
ros2 launch telemetry_recorder recorder.launch.py \
  experiment_name:=scene1_run_01

# Scene 2
ros2 launch telemetry_recorder recorder.launch.py \
  topics_config:=$(ros2 pkg prefix telemetry_recorder)/share/telemetry_recorder/config/fleet_topics.yaml \
  experiment_name:=scene2_run_01
```

Output goes to `~/field_robots_lab_experiments/<experiment_name>/`:

```
my_first_run/
├── bag/
│   ├── bag_0.mcap.zstd          # compressed mcap rosbag
│   └── metadata.yaml             # rosbag2-generated info
└── metadata.yaml                 # experiment metadata (start, stop, topics)
└── topics_config.yaml            # snapshot of topics config at record time
```


### Analyze the recording

```bash
ros2 run telemetry_recorder analyzer \
  ~/field_robots_lab_experiments/my_run_01
```

The analyzer reads the bag, computes per-topic timing metrics (actual
rate, inter-arrival time statistics, gap counts), compares against
`expected_rate_hz` from `topics_config.yaml`, writes `metrics.json`
next to the bag, and prints a summary table.

The recorder does not subscribe to data topics — native rosbag2
handles writing, and analysis runs offline. This keeps the recorder
out of the critical simulation path.

## Current progress

- **Scene 1 (Scout Mini in warehouse):** working end-to-end.
- **Scene 2 (heterogeneous fleet):** working. Three robots in one
  Gazebo instance under separate ROS2 namespaces with different
  sensor configurations.
- **telemetry_recorder:** functional. Native rosbag2, YAML-driven
  configuration, experiment metadata, clean SIGINT handling.
- **Post-hoc analyzer:** functional. Per-topic timing metrics with
  comparison against expected rates. Writes structured
  `metrics.json` per experiment.
- **scenario_runner:** functional. PD-controlled waypoint navigation
  per robot, orchestrated launches for full repeatable runs. No
  teleoperation required.
- **Scenes 3–4:** not started.

### Observations

**Scene 1 baseline (single robot).** Quiet run on a CPU-only Ubuntu
22.04 / ROS2 Humble laptop. IMU runs at 95.7 Hz vs the expected 100 Hz
(rate deviation around -4%). The p99 of IMU inter-arrival time settles
around 18–20 ms, roughly twice the expected 10 ms.

**Scene 2 baseline (three robots, teleop).** The same hardware now
runs three robots in one Gazebo. All sensor topics degrade uniformly
by ~9% — IMU at ~87 Hz, odometry at ~43.8 Hz, lidar at ~8.7 Hz across
all three robots. The degradation is uniform regardless of per-robot
sensor configuration: `robot_2` (IMU only) shows the same IMU rate
as `robot_0` (full sensor suite), indicating the bottleneck is at the
simulator/CPU level, not the per-robot configuration.

**Scene 2 automated (three robots, PD navigation).** With the scenario
runner driving all three robots through fixed waypoints via PD
control, telemetry stabilises further — IMU around 91–92 Hz, odometry
around 45–46 Hz, with substantially fewer gaps than under bursty
teleop. This is the working baseline against which load-injected and
anomaly scenarios will be compared.

### Repeatability check

Two independent runs of `scenario_navigation.launch.py` on the same
hardware:

| Metric                 | run_02   | run_03   | Δ       |
|------------------------|----------|----------|---------|
| robot_0 IMU rate (Hz)  | 92.51    | 90.51    | -2.2%   |
| robot_1 IMU rate (Hz)  | 91.78    | 91.72    | -0.07%  |
| robot_2 IMU rate (Hz)  | 91.12    | 90.97    | -0.16%  |
| robot_0 IMU p99 (ms)   | 17.24    | 18.26    | +5.9%   |
| robot_1 IMU p99 (ms)   | 17.30    | 17.95    | +3.8%   |
| robot_2 IMU p99 (ms)   | 17.36    | 17.92    | +3.2%   |
| robot_0 odom rate (Hz) | 46.29    | 45.27    | -2.2%   |
| robot_1 odom rate (Hz) | 45.92    | 45.95    | +0.06%  |
| robot_2 odom rate (Hz) | 45.59    | 45.53    | -0.13%  |
| cmd_vel rate (Hz)      | 19.48–19.57 | 19.49–19.57 | ~0% |

Rates reproduce within ±2.2%; tail latencies within ±6%. The PD
controller produces a deterministic `cmd_vel` stream; downstream
variability comes from Gazebo physics and sensor noise. This is the
quantitative baseline for any future anomaly detection work.

## Known limitations

- **`/tf` gap detection is misleading.** Gazebo publishes several TF
  messages per simulation step (one per moving link), so inter-arrival
  times are strongly bimodal: many sub-millisecond intervals within a
  burst, then a long pause until the next burst. The current
  median × 3 gap heuristic flags the long pauses as gaps even when
  the system is healthy. A multi-frame-aware metric is planned.
- **Sparse topics trigger spurious gaps.** Topics without a fixed
  publication rate (e.g. teleoperated `cmd_vel`) report misleading
  gap counts. Gap detection will be made opt-in per topic.
- **Single-machine simulation only.** All transport happens inside
  one host; cross-machine network effects are not yet exercised.
- **No load injection yet.** Scenes are observed in their natural
  load conditions. Controlled load profiles (CPU throttling,
  injected jitter, extra subscribers) are the next experimental
  layer.
- **Deterministic seeds not yet fixed.** Sensor noise uses
  Gazebo's default seeding; repeatability is statistical (see table
  above), not bit-identical.

## On Gazebo Classic and EOL

Gazebo Classic 11 reached end-of-life in January 2025. This repository
intentionally targets Classic for the following reasons:

- Stable, well-documented integration with ROS2 Humble (LTS until May 2027)
- Lower resource requirements — important for development and reproduction
  on machines without dedicated GPUs
- Existing ecosystem of robot models, plugins, and worlds remains the largest

Migration to Gazebo Harmonic/Ionic is planned as a separate effort once
all four scenes are stable on Classic. The xacro structure here is
written with that migration in mind: physical descriptions are isolated
from Gazebo-specific blocks (see scout_mini.gazebo.xacro vs
scout_mini.urdf.xacro), so only the latter needs to be rewritten.

## License

Apache License 2.0
