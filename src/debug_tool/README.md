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
  -p nav_state_topic:=/navigation/state \
  -p nav_task_points_topic:=/navigation/task_points \
  -p arm_mission_service:=/arm/mission_event \
  -p nav_arm_event_service:=/navigation/arm_event \
  -p nav_mission_request_topic:=/debug/nav/mission_request \
  -p nav_mission_response_topic:=/debug/nav/mission_response \
  -p nav_arm_event_request_topic:=/debug/nav/arm_event_request \
  -p nav_arm_event_response_topic:=/debug/nav/arm_event_response \
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
- latest nav pose from `/navigation/state`
- latest nav task-point snapshot from `/navigation/task_points`
- availability of common arm services from `robot_msgs`
- availability of `/arm/mission_event` and `/navigation/arm_event`
- latest mirrored `/arm/mission_event` request/response debug topics
- latest mirrored `/navigation/arm_event` request/response debug topics
- availability of the `move_joint` action server

CSV logging is enabled by default. Each run creates one file named with the node startup time:

```text
debug_tool_logs/YYYYMMDD_HHMMSS.csv
```

Each row contains a wall-clock timestamp, ROS time, elapsed time, driver ready state, service/action availability, joint feedback, and command fields:

- nav integration: `nav_state_x`, `nav_state_y`, `nav_state_yaw`, `nav_task_points_*`
- nav mission tracing: `nav_mission_*`, `nav_arm_event_*`
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

## Navigation Mock UI

`nav_mock_ui` is a Qt tool that replaces the nav package and real lidar for arm-side debugging without modifying the external `navigation` package.

It provides:

- a mock `/navigation/state` odometry publisher
- a mock `/navigation/task_points` publisher
- a client for `/arm/mission_event`
- a server for `/navigation/arm_event`
- an optional mock `get_pick_pos` server for nav pickup debugging without the real vision node
- a 2D map view for task points, mock pose, and `/task/target_pose`
- subscriptions to `/debug/nav/*` mirror topics published by `arm2_task`

Run:

```bash
source install/setup.bash
ros2 run debug_tool nav_mock_ui
```

Or through launch:

```bash
ros2 launch debug_tool nav_mock_ui.launch.py
```

Useful parameters:

```bash
ros2 run debug_tool nav_mock_ui --ros-args \
  -p nav_state_topic:=/navigation/state \
  -p nav_task_points_topic:=/navigation/task_points \
  -p arm_mission_service:=/arm/mission_event \
  -p nav_arm_event_service:=/navigation/arm_event \
  -p pose_auto_publish:=true \
  -p pose_publish_hz:=10.0 \
  -p vision_override_enabled:=false \
  -p vision_pick_z:=0.12
```

Interaction notes:

- left-click a task point on the map to select it
- right-click the map to move the mock lidar pose in `x/y`
- edit pose, task points, and mission fields from the right panel
- enable `Mock Vision` to make `get_pick_pos` return the nearest task point converted into arm `world` coordinates
- incoming arm callbacks are answered automatically using the configured response fields
