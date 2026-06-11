# Documentation
# Clearpath Husky multi-robot patches for ROS 2 Humble

These patches enable spawning multiple Husky robots in a single Gazebo Classic 11 session with namespaced topics. The stock Husky URDF (`humble-devel` branch) does not support multi-robot setup out of the box.

## Apply

Clone Husky source:

```bash
mkdir -p ~/husky_ws/src && cd ~/husky_ws/src
git clone -b humble-devel https://github.com/husky/husky.git
```

Apply patches:

```bash
cd ~/husky_ws/src/husky/husky_description/urdf
patch < /path/to/01_decorations.patch
patch < /path/to/02_husky_macro.patch
patch < /path/to/03_husky_main.patch
patch < /path/to/04_wheel.patch
```

Build:

```bash
cd ~/husky_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## What each patch does

### 01_decorations.patch

Adds `prefix` parameter to `husky_decorate` macro and propagates it to all links and joints (`top_chassis`, `bumpers`, `top_plate`, `user_rail`). Without this, accessories use unprefixed `base_link` and the URDF fails to parse for non-empty prefix.

### 02_husky_macro.patch

Passes `prefix` argument to `husky_decorate` macro call and to `husky_wheel` macro calls. Wraps the `<ros2_control>` block in `<xacro:unless value="$(arg is_sim)">` so it is only active for real hardware; in simulation we use `libgazebo_ros_diff_drive.so` directly.

### 03_husky_main.patch

Adds `robot_namespace` xacro arg (without trailing slash, for use in `<ros><namespace>` blocks where ROS 2 rejects trailing slashes). Replaces `gazebo_ros2_control` plugin with `gazebo_ros_diff_drive` (skid-steer pattern with `num_wheel_pairs=2`). Removes `$(arg prefix)` from plugin names (ROS 2 forbids `/` in node names).

### 04_wheel.patch

Adds `prefix` parameter to `husky_wheel` macro so wheel joints reference `${prefix}base_link` instead of unprefixed `base_link`.

## Why not `gazebo_ros2_control`?

`libgazebo_ros2_control.so` exhibits singleton behaviour in Gazebo Classic 11: loading the plugin a second time (for a second robot) reuses state from the first, causing controllers to fail with `Skipping joint ... which is not in the gazebo model`.

`libgazebo_ros_diff_drive.so` instantiates per-robot cleanly, which is the same pattern Clearpath uses for AgileX Scout Mini and similar wheeled robots.