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
| 3 | DJI M350-class surrogate (PX4 SITL) | pipeline inspection sweep | working — see Quick Start |
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
- **For Scene 3 only:** PX4 SITL v1.16.2, Micro-XRCE-DDS Agent v3.0.0,
  `px4_msgs` v1.16.2

Other simulators may be added where they fit better (e.g., MuJoCo for
quadruped contact dynamics) — noted per scene.

## Repository structure

```
field_robots_lab/
├── src/
│   ├── scout_mini_description/   # Scout Mini URDF, sensor macros, launch
│   ├── telemetry_recorder/        # Recording wrapper + analyzer
│   ├── scenario_runner/           # Scout Mini PD-controlled scenarios
│   ├── drone_scenario_runner/     # PX4 offboard waypoint missions
│   └── field_robots_worlds/       # Gazebo worlds + shared materials/textures
├── docs/                          # Per-scene documentation (planned)
└── README.md
```

Worlds and visual assets (textures, material scripts) are decoupled
into a separate `field_robots_worlds` package so they can be reused
across robots and scenes. Sensors on Scout Mini are isolated into
parameterized xacro macros (`urdf/sensors/imu.xacro`,
`urdf/sensors/lidar_2d.xacro`); each robot instance can be configured
with different sensor profiles without touching the base URDF. The
main robot xacro accepts `use_imu`, `use_lidar`, `lidar_samples`,
`imu_rate`, and `namespace` as launch-supplied arguments.

For Scene 3, the PX4 autopilot and Micro-XRCE-DDS Agent live outside
this workspace (in `~/PX4-Autopilot/` and `~/Micro-XRCE-DDS-Agent/`
by convention). The `px4_msgs` ROS2 package lives in a separate
workspace (`~/px4_ros_ws/`) to keep PX4 message generation isolated
from this project's build.

## Quick Start

Requires Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11 +
`gazebo_ros_pkgs`. Scene 3 also requires PX4 SITL — see *Scene 3
setup* below.

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

### Run a repeatable Scout Mini scenario (Scene 2 with PD navigation)

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
using a PD controller. No teleoperation. A typical run takes 60–90
seconds.

### Manual driving (Scenes 1 and 2 without scenarios)

```bash
# Scene 1
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.2}}"

# Scene 2: target a specific robot
ros2 topic pub --rate 10 /robot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.1}}"
```

### Run Scene 3: PX4 SITL drone mission

Scene 3 requires PX4 SITL to be installed separately. See *Scene 3
setup* below for the first-time setup procedure.

Once PX4 is built and the Micro-XRCE-DDS Agent is installed, a
repeatable drone mission run uses two terminals:

```bash
# Terminal 1: PX4 SITL + Gazebo with M350-class airframe
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=10025 make px4_sitl_default gazebo-classic

# Wait for "Ready for takeoff!" prompt before launching the mission.
```

```bash
# Terminal 2: orchestrated mission (Agent + recorder + navigator)
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/field_robots_lab/install/setup.bash

ros2 launch drone_scenario_runner drone_mission.launch.py \
  experiment_name:=drone_pipeline_sweep_01
```

The launch starts:
- Micro-XRCE-DDS Agent on UDP port 8888
- Telemetry recorder writing to
  `~/field_robots_lab_experiments/drone_pipeline_sweep_01/`
- `drone_waypoint_navigator` — sends PX4 commands to arm, switch to
  offboard mode, fly a 5-waypoint inspection sweep at 5 m altitude
  along the pipeline corridor (~80 m total path), then auto-land

A typical mission takes ~60–90 seconds. The navigator exits
automatically after landing.

To use the default iris airframe instead of the M350-class one, omit
`PX4_SYS_AUTOSTART=10025` from the PX4 launch command. The mission
itself does not depend on which airframe is loaded — the navigator
issues NED position setpoints that any multicopter can follow.

### Drone telemetry without a mission (baseline)

For comparison runs (drone hovering or driven via PX4 commander
without offboard automation), record telemetry directly:

```bash
# Terminal 1: PX4 SITL (as above)

# Terminal 2: MicroXRCEAgent + recorder
MicroXRCEAgent udp4 -p 8888 &
sleep 2
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/field_robots_lab/install/setup.bash
ros2 launch telemetry_recorder recorder.launch.py \
  topics_config:=$(ros2 pkg prefix telemetry_recorder)/share/telemetry_recorder/config/drone_topics.yaml \
  experiment_name:=drone_baseline_01

# Terminal 3 (in Terminal 1's PX4 shell):
pxh> commander takeoff
pxh> commander land
```

### Pipeline worlds (Scene 3 environments)

Two worlds are available in `field_robots_worlds`, both using
textured ground for outdoor realism:

- **`pipeline_above_ground.world`** — pipe rack with four parallel
  pipes on supports, valves, and boundary fences. 60×30 m corridor.
- **`pipeline_underground_surface.world`** — outdoor scene representing
  a buried pipeline corridor as seen from a surface inspection drone:
  pipeline route markers, anomaly patches (soil discoloration,
  subsidence, vegetation stress) against a textured ground with
  normal vegetation reference zones.

Launch a world standalone with Gazebo (no drone):

```bash
gazebo $(ros2 pkg prefix field_robots_worlds)/share/field_robots_worlds/worlds/pipeline_above_ground.world
```

The current Scene 3 mission flies in the default empty PX4 world. The
mission trajectory uses NED coordinates corresponding to the
above-ground pipeline corridor extent (x ∈ [-20, +20] at altitude 5 m).
Integrating PX4 SITL spawn with the field_robots_worlds pipeline
worlds is a future refinement; the mission flight path is already
laid out for that geometry.

### Record telemetry separately (without scenario orchestration)

The recorder can be used standalone with any of the launches above.
Scene 1 uses `scout_mini_topics.yaml` (default); Scene 2 uses
`fleet_topics.yaml`; Scene 3 uses `drone_topics.yaml`:

```bash
# Scene 1
ros2 launch telemetry_recorder recorder.launch.py \
  experiment_name:=scene1_run_01

# Scene 2
ros2 launch telemetry_recorder recorder.launch.py \
  topics_config:=$(ros2 pkg prefix telemetry_recorder)/share/telemetry_recorder/config/fleet_topics.yaml \
  experiment_name:=scene2_run_01

# Scene 3 baseline (without mission)
ros2 launch telemetry_recorder recorder.launch.py \
  topics_config:=$(ros2 pkg prefix telemetry_recorder)/share/telemetry_recorder/config/drone_topics.yaml \
  experiment_name:=scene3_run_01
```

Output goes to `~/field_robots_lab_experiments/<experiment_name>/`:

```
my_run_01/
├── bag/
│   ├── bag_0.mcap                # native mcap rosbag
│   └── metadata.yaml             # rosbag2-generated info
├── metadata.yaml                 # experiment metadata (start, stop, topics)
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

## Scene 3 setup (PX4 SITL)

Scene 3 requires several external components installed once:

```bash
# 1. PX4-Autopilot v1.16.2
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.16.2
git submodule update --init --recursive

# Install dependencies (skipping NuttX and bundled simulators)
bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# Additional packages required for Gazebo Classic build:
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                 libunwind-dev

# Build PX4 SITL with GCC (clang-18 is incompatible with v1.16.2)
make distclean
CC=gcc CXX=g++ make px4_sitl_default gazebo-classic
```

```bash
# 2. Micro-XRCE-DDS Agent v3.0.0 (ROS2 ↔ PX4 bridge)
cd ~
git clone -b v3.0.0 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

```bash
# 3. px4_msgs v1.16.2 in a separate workspace
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout v1.16.2
cd ~/px4_ros_ws
source /opt/ros/humble/setup.bash
colcon build
```

```bash
# 4. Install the M350-class airframe in PX4
# Copy the airframe file (defines mass, max speed, hover throttle, etc.)
# from this repo into the PX4 ROMFS, then register it in the CMakeLists.
# See docs/scene3_m350_airframe.md for the airframe contents (planned).
```

Total disk usage for Scene 3 dependencies: approximately 5 GB.

## Current progress

- **Scene 1 (Scout Mini in warehouse):** working end-to-end.
- **Scene 2 (heterogeneous fleet):** working. Three robots in one
  Gazebo instance under separate ROS2 namespaces with different
  sensor configurations.
- **scenario_runner:** functional. PD-controlled waypoint navigation
  per robot, orchestrated launches for full repeatable runs. No
  teleoperation required.
- **Scene 3 (PX4 SITL drone):** working. Drone takes off, flies a
  fixed waypoint sweep in offboard mode, and auto-lands — all through
  a single `ros2 launch` command. M350-class airframe parameters
  available via `PX4_SYS_AUTOSTART=10025`. Telemetry recorded
  end-to-end through the Micro-XRCE-DDS Agent.
- **drone_scenario_runner:** functional. PX4 offboard control via
  `px4_msgs`, position-based waypoint navigation in NED frame,
  auto-arm, auto-land, automated mission termination.
- **Pipeline worlds:** above-ground (pipe rack) and underground-surface
  (corridor with anomaly patches) worlds available in
  `field_robots_worlds`. Both use textured ground; the underground
  world additionally uses textured vegetation patches.
- **telemetry_recorder:** functional. Native rosbag2, YAML-driven
  configuration, experiment metadata, clean SIGINT handling.
- **Post-hoc analyzer:** functional. Per-topic timing metrics with
  comparison against expected rates. Writes structured
  `metrics.json` per experiment.
- **Scene 4 (quadruped):** not started.

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
teleop.

**Scene 3 baseline (single PX4 drone, teleop).** With the drone taking
off, hovering, and landing under PX4 commander control (one-shot
commands from `pxh>` shell), all PX4 output topics serialize through
the uXRCE-DDS bridge at a uniform rate of approximately 86 Hz,
regardless of the rates at which PX4 publishes internally (which
differ per topic, e.g. 100 Hz for `vehicle_local_position` vs 250 Hz
nominal for `vehicle_attitude`). Gap counts on all PX4 topics are
uniformly high (~1860 gaps across ~13500 messages each), and p99
inter-arrival latency is ~27 ms.

**Scene 3 mission (single PX4 drone, offboard waypoints).** With the
drone flying a continuous offboard mission (5 waypoints, ~80 m path,
auto-arm and auto-land), the bridge behavior is dramatically
different. `vehicle_local_position`, `vehicle_attitude`,
`sensor_combined`, and `battery_status` all stabilise near **99.6 Hz**
— effectively the full expected rate. Gaps drop from ~1860 to **1–2
per topic**, and IMU p99 latency drops from 27 ms to **14.5 ms**. This
indicates that the uXRCE-DDS bridge bottleneck observed in the
baseline is a function of the **control pattern** (bursty teleop
commands), not the bridge itself or the underlying hardware. The
mission's continuous offboard control stream evens out publication
patterns end-to-end. This is a system-level observation precisely of
the kind a forward latency model (cf.
[temporis_ros2](https://github.com/Hippythalamus/temporis_ros2))
should be able to predict.

### Repeatability check (Scene 2)

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
- **Bridge-bursted topics report misleading gaps.** PX4 topics
  delivered through the uXRCE-DDS bridge under teleop control share
  the same bimodal inter-arrival pattern. The same gap heuristic above
  misclassifies these gaps. Under offboard mission control this
  effectively disappears (~1 gap per topic), confirming that the
  pattern is control-driven rather than transport-driven. The
  heuristic will still be updated to handle the teleop case.
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
- **Scene 3 drone visual is unchanged.** The M350-class airframe
  changes flight parameters (mass, max speed, tilt limits, hover
  throttle, battery cells) but the visible Gazebo model remains the
  default iris quadrotor. A visual model swap is not part of the
  current scope.
- **Scene 3 mission runs in the default PX4 world.** The
  field_robots_worlds pipeline environments are loaded standalone
  for visual reference but not yet integrated with PX4 SITL spawn.
  The mission trajectory uses coordinates matching the above-ground
  corridor geometry, so the integration is purely a launch-side
  concern.

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