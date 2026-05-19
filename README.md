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
| 1 | Scout Mini (AgileX) | warehouse navigation baseline | in progress |
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

## Related work

This repository is part of a research arc around distributed robotics
reliability:

- [temporis_ros2](https://github.com/Hippythalamus/temporis_ros2) —
  calibrated transport latency model for ROS2/Zenoh systems

## License

Apache License 2.0
