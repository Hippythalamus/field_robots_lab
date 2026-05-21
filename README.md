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
