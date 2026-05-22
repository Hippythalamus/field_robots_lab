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
| 2 | Heterogeneous fleet | 2–3 platforms, coordination scenario | planned |
| 3 | DJI M350-class surrogate (PX4 SITL) | pipeline inspection | planned |
| 4 | Quadruped (Go2 / ANYmal-class) | indoor inspection | planned |

## What this is not

- Not a tutorial repo — assumes ROS2 working knowledge
- Not a Nav2/MoveIt showcase — scenes are deliberately minimal in autonomy
- Not a benchmark suite — scenes are research stands, not standardized tests
- Not production-ready — research prototype, see Limitations section

## Stack

- ROS2 Humble
- Gazebo Classic 11
- Ubuntu 22.04

Other simulators may be added where they fit better (e.g., MuJoCo for
quadruped contact dynamics) — noted per scene.

## Repository structure

field_robots_lab/
├── src/
│   ├── scout_mini_description/   # URDF, Gazebo plugins, world, launch
│   └── telemetry_recorder/        # Recording wrapper + metadata
├── docs/                          # Per-scene documentation (planned)
└── README.md

Scene-specific files (worlds, launch files) currently live inside the
robot description package. They will be reorganized into a top-level
`scenes/` directory once a second scene is added.

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
# Terminal 1 — launch Gazebo with the robot
ros2 launch scout_mini_description gazebo.launch.py
```

This spawns a Scout Mini in a 20×20 m enclosed warehouse with six static
obstacles. Wait ~30 seconds for Gazebo to load (longer on CPU-only
machines). The robot publishes:

- `/imu` at 100 Hz
- `/scan` (2D lidar) at 10 Hz, 360°, 0.12–12 m range
- `/odom` at 50 Hz from skid-steer plugin
- `/joint_states` for wheel positions
- `/tf`, `/tf_static`

### Drive the robot

```bash
# Terminal 2 — keyboard teleop (hold keys)
sudo apt install ros-humble-teleop-twist-keyboard  # if not installed
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or publish to `/cmd_vel` directly:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.2}}"
```

### Record telemetry

```bash
# Terminal 3 — record an experiment
ros2 launch telemetry_recorder recorder.launch.py \
  experiment_name:=my_first_run
```

The recorder spawns `ros2 bag record` as a subprocess (no Python overhead
on data streams) and writes experiment metadata alongside the bag.

Output goes to `~/field_robots_lab_experiments/<experiment_name>/`:

my_first_run/
├── bag/
│   ├── bag_0.mcap.zstd          # compressed mcap rosbag
│   └── metadata.yaml             # rosbag2-generated info
└── metadata.yaml                 # experiment metadata (start, stop, topics)

Stop the recorder with `Ctrl+C`. The metadata file is finalized with
the stop timestamp.

### Inspect a recording

```bash
ros2 bag info ~/field_robots_lab_experiments/my_first_run/bag
cat ~/field_robots_lab_experiments/my_first_run/metadata.yaml
```

## Current progress

- **Scene 1 (Scout Mini in warehouse):** working end-to-end. Robot
  spawns, drives, sensors publish, recorder produces valid mcap bags
  with experiment metadata.
- **telemetry_recorder:** minimal but functional. Records via native
  rosbag2 (no per-message Python work), writes experiment metadata,
  handles `Ctrl+C` cleanly.
- **Post-hoc analyzer:** next milestone. Reads recorded bags and
  produces per-topic metrics (actual rates, inter-arrival time
  statistics, gaps).
- **Scenes 2–4:** not started.

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
