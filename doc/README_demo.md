# rokae_example demo launch notes

This document covers the common demos under `rokae_example/launch`:

- `joint_s_line.launch.py`
- `cartesian_s_line.launch.py`
- `follow_joint_position.launch.py`
- `follow_cart_position.launch.py`
- `servoj_demo.launch.py`

> Note: `joint_s_line_rt.launch.py` is currently an empty file and cannot be launched.

## 1. Prerequisites

- ROS 2 is installed (matching this workspace)
- The workspace contains and can build:
  - `rokae_hardware`
  - `rokae_example`
  - `rokae_description`
  - `rokae_xMate<model>_moveit_config` (for example `rokae_xMateCR7_moveit_config`, `rokae_xMateCR35_moveit_config`)
- The robot model must be in the supported list:
  - `CR7/CR12/CR18/CR20/CR35/ER3/ER7/SR3/SR4/SR5/Pro3/Pro7/AR5L/AR5R`

## 2. Build and source (bash)

From the workspace root (`ros2_ws`):

```bash
colcon build --packages-select rokae_hardware rokae_example rokae_description
```

## 3. Recommended run order (two terminals)

The `ros2 launch rokae_example ...` examples below depend on the full stack already running: **`move_group`**, **`/joint_states`**, the **trajectory controller**, and so on. Follow this order.

**Terminal 1 (start the full stack first and keep it running)**

Start the full launch provided by `rokae_hardware` (`ros2_control`, MoveIt, `move_group`, RViz, etc. come up with it):

```bash
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=<model> use_fake_hardware:=false robot_ip:=<controller_ip> local_ip:=<local_nic_ip>
```

- Replace `<model>` with your model, for example `CR7` or `CR35`.
- **Software chain and RViz only:** you can use `use_fake_hardware:=true` (IPs can still be filled in).
- **Real robot and HMI:** use `use_fake_hardware:=false` and set `robot_ip` / `local_ip` to the controller and this PC.

After terminal 1 has no fatal errors and Motion Planning works in RViz, open **terminal 2** (also `source install/setup.bash` or the equivalent) and run the demo commands in section 4.

**Terminal 2 (only after the full stack is stable)**

- Run demos from **`rokae_example`**: `ros2 launch rokae_example <launch>.py robot_type:=<model> ...`.
- To use **`controll_movej.launch.py`** (the `movej` node in `rokae_hardware`), also keep terminal 1 running and in terminal 2 run:

```bash
ros2 launch rokae_hardware controll_movej.launch.py robot_type:=<model>
```

Do **not** expect the MoveIt demos in section 4 to plan and execute if the full stack from terminal 1 is not running.

## 4. How to launch each demo

Run these in a bash shell that has already sourced the workspace. **Complete section 3 terminal 1 first.**

### 4.1 `joint_s_line` (joint-space S-curve)

Minimal launch (CR7 example):

```bash
ros2 launch rokae_example joint_s_line.launch.py robot_type:=CR7
```

xMate CR35 example:

```bash
ros2 launch rokae_example joint_s_line.launch.py robot_type:=CR35
```

Optional arguments:

- `robot_type`: model, default `CR7`

---

### 4.2 `cartesian_s_line` (Cartesian S-curve)

Minimal launch:

```bash
ros2 launch rokae_example cartesian_s_line.launch.py robot_type:=CR7
```

xMate CR35 example:

```bash
ros2 launch rokae_example cartesian_s_line.launch.py robot_type:=CR35
```

Optional arguments:

- `robot_type`: model, default `CR7`

---

### 4.3 `follow_joint_position` (joint waypoint following)

Minimal launch:

```bash
ros2 launch rokae_example follow_joint_position.launch.py robot_type:=CR7
```

Common arguments:

```bash
ros2 launch rokae_example follow_joint_position.launch.py robot_type:=CR7 point_period_s:=0.6 start_blend_s:=0.6 goal_tolerance_rad:=0.01 vel_scale:=0.2 acc_scale:=0.2 cycle_count:=0
```

Argument notes:

- `point_period_s`: time between adjacent waypoints (seconds)
- `start_blend_s`: blend time reserved for the first point (seconds)
- `goal_tolerance_rad`: joint goal tolerance (radians)
- `vel_scale`: velocity scaling $(0,1]$
- `acc_scale`: acceleration scaling $(0,1]$
- `cycle_count`: round-trip cycles; `0` means infinite

---

### 4.4 `follow_cart_position` (Cartesian sine following)

Minimal launch:

```bash
ros2 launch rokae_example follow_cart_position.launch.py robot_type:=CR7
```

Common arguments:

```bash
ros2 launch rokae_example follow_cart_position.launch.py robot_type:=CR7 amplitude_m:=0.4 period_s:=1.33 target_update_hz:=1.0 control_hz:=5.0 kp:=0.6 filter_alpha:=0.25 duration_s:=20.0 vel_scale:=0.08 acc_scale:=0.08 eef_step:=0.002 min_fraction:=0.95
```

Argument notes:

- `amplitude_m`: Y-axis oscillation amplitude (meters)
- `period_s`: sine period (seconds)
- `target_update_hz`: target-point update rate (Hz)
- `control_hz`: control-step execution rate (Hz)
- `kp`: proportional tracking gain
- `filter_alpha`: first-order filter coefficient $(0,1]$
- `duration_s`: total duration (seconds)
- `vel_scale`: velocity scaling $(0,1]$
- `acc_scale`: acceleration scaling $(0,1]$
- `eef_step`: Cartesian path interpolation step (meters)
- `min_fraction`: minimum local-path completion fraction

---

### 4.5 `servoj_demo` (SDK ServoJ example)

Minimal launch:

```bash
ros2 launch rokae_example servoj_demo.launch.py
```

Network and motion arguments:

```bash
ros2 launch rokae_example servoj_demo.launch.py robot_ip:=192.168.21.10 local_ip:=192.168.21.131 movej_speed:=0.2 cycle_ms:=20 duration_s:=30.0 period_s:=4.0 servoj_lookahead_s:=0.06 servoj_kp:=1.0
```

Argument notes:

- `robot_ip`: robot controller IP
- `local_ip`: local NIC IP
- `movej_speed`: MoveJ velocity scaling $(0,1]$
- `cycle_ms`: ServoJ command period (milliseconds)
- `duration_s`: oscillation duration (seconds)
- `period_s`: cosine period (seconds)
- `servoj_lookahead_s`: ServoJ lookahead time
- `servoj_kp`: ServoJ gain

## 5. Troubleshooting

1. Cannot find `rokae_xMate<model>_moveit_config`:
   - Check the model spelling and that the package is built and installed. CR35 uses `rokae_xMateCR35_moveit_config`.
2. SRDF or `kinematics.yaml` not found:
   - Confirm the corresponding model config package has a complete `config` directory.
3. Command or package not found:
   - Rebuild with `colcon build` and `source install/setup.bash` (or the equivalent for your shell).
4. `servoj_demo` cannot communicate:
   - Check `robot_ip`, `local_ip`, subnet, and firewall settings.
