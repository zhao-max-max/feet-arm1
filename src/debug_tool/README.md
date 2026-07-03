# debug_tool

`debug_tool` is a small ROS2 C++ package for checking the arm debug interfaces in this workspace.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select debug_tool
```

## Run

```bash
source install/setup.bash
ros2 run debug_tool debug_tool_node
```

Or start it through launch:

```bash
ros2 launch debug_tool debug_tool_node.launch.py
```

Useful parameters:

```bash
ros2 run debug_tool debug_tool_node --ros-args \
  -p state_topic:=/arm2/_lowState/joint \
  -p command_topic:=/arm2/_lowCmd/command \
  -p ready_topic:=/robot_driver/ready \
  -p report_period_sec:=1.0 \
  -p csv_enabled:=true \
  -p csv_dir:=debug_tool_logs \
  -p motor_count:=5
```

Launch example:

```bash
ros2 launch debug_tool debug_tool_node.launch.py \
  report_period_sec:=0.5 \
  csv_dir:=debug_tool_logs
```

The node prints:

- driver ready state from `/robot_driver/ready`
- current joint position in degrees and velocity from `/arm2/_lowState/joint`
- latest angle control command from `/arm2/_lowCmd/command`
- availability of common arm services from `robot_msgs`
- availability of the `move_joint` action server

CSV logging is enabled by default. Each run creates one file named with the node startup time:

```text
debug_tool_logs/YYYYMMDD_HHMMSS.csv
```

Each row contains a wall-clock timestamp, ROS time, elapsed time, driver ready state, service/action availability, joint feedback, and command fields:

- feedback: `state_q_rad_*`, `state_q_deg_*`, `state_dq_*`, `state_tau_est_*`, `state_valid_*`
- command: `cmd_q_rad_*`, `cmd_q_deg_*`, `cmd_dq_*`, `cmd_tau_*`, `cmd_kp_*`, `cmd_kd_*`

## Joint Slider UI

The Qt UI controls the arm through `control_node`'s `move_joint` action server. It does not publish directly to the low-level command topic.

```bash
source install/setup.bash
ros2 run debug_tool joint_slider_ui
```

Or start it through launch:

```bash
ros2 launch debug_tool joint_slider_ui.launch.py
```

Controls:

- `参数模式`: selects the `control_node` gain mode sent through `set_controller_mode`.
- `切换参数模式`: sends the selected mode immediately without moving the sliders.
- `运动开关`: enables or disables sending `move_joint` goals. Turning it off also requests cancellation of the active goal.
- `同步当前关节角`: copies the latest `/arm2/_lowState/joint` positions into the five sliders.
- five sliders/spin boxes: joint targets in degrees. While motion is enabled, slider changes are debounced and sent as single-point `move_joint` goals.

Useful parameters:

```bash
ros2 run debug_tool joint_slider_ui --ros-args \
  -p default_mode:=moving \
  -p max_velocity:=0.6 \
  -p max_acceleration:=1.0 \
  -p blend_radius:=0.02 \
  -p send_debounce_ms:=180 \
  -p joint_lower_deg:="[-240.0, 0.0, -172.0, -115.0, -180.0]" \
  -p joint_upper_deg:="[240.0, 200.0, 10.0, 90.0, 180.0]"
```
