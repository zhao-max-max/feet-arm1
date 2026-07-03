# nav 与 arm2_task 导航抓取链路代码检查记录

检查日期：2026-07-03

检查范围：

- `src/nav/launch/navigation.launch.py`
- `src/nav/src/app/navigation_runtime.cpp`
- `src/nav/src/app/navigation_map_node.cpp`
- `src/nav/src/app/navigation_ui_coordinator.cpp`
- `src/nav/src/interface/radar_interface.cpp`
- `src/nav/srv/MissionCommand.srv`
- `src/nav/msg/MapPoint.msg`
- `src/nav/msg/MapPointArray.msg`
- `src/nav/config/points/task_points.yaml`
- `src/arm2_task/src/task/nav_task_interface.cpp`
- `src/arm2_task/src/task/nav_pose_tracker.cpp`
- `src/arm2_task/src/task/task_primitives.cpp`
- `src/arm2_task/src/task/task_sequences.cpp`
- `src/arm2_task/src/task_node.cpp`
- `src/arm2_task/config/task_params.yaml`
- `src/arm2_task/launch/task_node.launch.py`

验证结果：

- `colcon build --packages-select navigation arm2_task` 通过。
- 本文结论只基于当前代码，不基于历史记忆或旧日志。

## 1. 当前整体链路

### 1.1 nav 启动与 mission 模式

`navigation.launch.py` 默认启动 `navigation_map`：

```python
navigation_executable_arg = DeclareLaunchArgument("navigation_executable", default_value="navigation_map")
```

但 `race_logic` 默认是 `obstacle`：

```python
race_logic_arg = DeclareLaunchArgument("race_logic", default_value="obstacle")
```

因此要跑导航抓取完整 mission，需要启动 nav 时显式设置：

```bash
ros2 launch navigation navigation.launch.py race_logic:=mission
```

nav 默认与机械臂交互的两个服务名为：

- `/arm/mission_event`：nav 调用机械臂，发送 `ready/pickup/place`。
- `/navigation/arm_event`：机械臂调用 nav，反馈 `grabbed/placed/completed`。

对应参数在 `navigation.launch.py` 中为：

```python
arm_mission_service_arg = DeclareLaunchArgument("arm_mission_service", default_value="/arm/mission_event")
navigation_arm_event_service_arg = DeclareLaunchArgument(
    "navigation_arm_event_service",
    default_value="/navigation/arm_event",
)
```

### 1.2 nav 发布静态任务点

nav 会发布 `/navigation/task_points`，类型为 `navigation/msg/MapPointArray`。

消息定义：

```text
std_msgs/Header header
MapPoint[] points
```

单个点 `MapPoint` 包含：

```text
int32 id
float64 x
float64 y
uint8 task_type
string event_label
```

默认任务点文件是：

```text
src/nav/config/points/task_points.yaml
```

当前文件中存在 `id=1..12` 的静态任务点。机械臂侧 `ready.task_index` 可以用这些 id 查到静态地图坐标。

### 1.3 nav 发布机器人状态

nav 会发布 `/navigation/state`，类型为 `nav_msgs/Odometry`。

默认 `navigation_map_node.cpp` 中发布状态时：

```cpp
msg.header.frame_id = state.frame_id.empty() ? "map" : state.frame_id;
msg.child_frame_id = "base_link";
msg.pose.pose.position.x = state.x;
msg.pose.pose.position.y = state.y;
msg.pose.pose.orientation = quaternionFromYaw(state.yaw);
```

雷达输入来自 `radar_interface.cpp` 中的 `/Odometry`：

```cpp
const auto topic = node.declare_parameter<std::string>("radar_odom_topic", "/Odometry");
subscription_ = node.create_subscription<nav_msgs::msg::Odometry>(
  topic,
  rclcpp::SensorDataQoS(),
  ...
);
```

雷达接口会根据 nav 自己的标定文件对输入 odometry 做二维旋转、平移和 yaw offset，但没有使用机械臂与雷达之间的外参。

## 2. mission 任务生成逻辑

### 2.1 pickup/place 类型判断

`navigation_runtime.cpp` 中 `taskTypeForMissionPoint()` 的逻辑是：

- 如果路径点显式设置 `task_type=pickup/place`，直接使用。
- 如果路径点没有显式任务类型但 `fast=true`，按 mission 任务序号交替推断 pickup/place。
- 其他点不是机械臂任务点。

对应逻辑：

```cpp
if (point.task_type == navigation::maps::kTaskTypePickup ||
  point.task_type == navigation::maps::kTaskTypePlace)
{
  return point.task_type;
}

if (point.fast) {
  return task_index % 2 == 0 ? navigation::maps::kTaskTypePickup : navigation::maps::kTaskTypePlace;
}

return navigation::maps::kTaskTypeNone;
```

### 2.2 mission_target_id 来源

UI 生成 mission 路径时，会把静态目标 id 写入路径点的 `event_label`：

```cpp
map_point.event_label = missionTargetLabel(mission_target_id);
```

格式为：

```text
@mission_target_<id>
```

runtime 重置 mission 任务时解析该字段：

```cpp
task.mission_target_id = parseMissionTargetId(points[i].event_label);
```

这说明 `mission_target_id` 才是静态地图任务点 id，范围应对应 `/navigation/task_points` 中的 `1..12`。

## 3. nav 到机械臂的服务调用

### 3.1 普通到点 pickup/place

机器狗到达任务点半径内后，`NavigationRuntime::handleMissionArrival()` 会暂停导航：

```cpp
task.triggered = true;
context_.mission_paused = true;
context_.mission_current_task = next_task_index;
```

随后 `sendArrivedToArmIfDue()` 调用 `/arm/mission_event`：

```cpp
request->task_index = static_cast<std::uint32_t>(context_.mission_current_task);
request->point_id = task.point_id;
request->action = taskActionText(task.task_type);
request->x = point.x;
request->y = point.y;
```

这里字段含义是：

- `task_index`：mission 队列中的序号，不是静态任务点 id。
- `point_id`：导航路径点 id，不是静态任务点 id。
- `action`：`pickup` 或 `place`。
- `x/y`：导航路径点坐标，不一定等于箱子或放置区的静态任务点坐标。

该请求有 retry，周期由 `mission_arm_retry_period` 控制：

```cpp
if (elapsed < context_.mission_arm_retry_period) {
  return;
}
```

### 3.2 completed 后的 ready

机械臂调用 `/navigation/arm_event` 发送 `completed` 后，nav 会在 `handleArmEvent()` 中调用：

```cpp
sendReadyForNextMissionTask(event_task_index);
```

`sendReadyForNextMissionTask()` 会给下一个 mission 任务发送一次 `ready`：

```cpp
request->task_index = static_cast<std::uint32_t>(next_task.mission_target_id);
request->point_id = next_task.point_id;
request->action = "ready";
request->x = point.x;
request->y = point.y;
```

这里字段含义是：

- `task_index`：下一个任务的静态目标 id，也就是 `/navigation/task_points` 里的 `id`。
- `point_id`：下一个导航路径点 id。
- `action`：固定为 `ready`。
- `x/y`：下一个导航路径点坐标，不是静态任务点坐标。

这个设计与普通 `pickup/place` 不同。机械臂侧应该只在 `ready` 中把 `task_index` 当静态任务点 id 使用。

## 4. arm2_task 当前处理逻辑

### 4.1 /arm/mission_event 服务

机械臂侧 `NavTaskInterface` 创建服务：

```cpp
arm_mission_server_ = node_->create_service<navigation::srv::MissionCommand>(
  "/arm/mission_event",
  ...
);
```

允许的 action：

```cpp
request->action != "ready" && request->action != "pickup" && request->action != "place"
```

当前状态检查：

- `ready` 只允许 `IDLE` 或 `LOOKOUT`。
- `pickup` 只允许 `IDLE` 或 `LOOKOUT`。
- `place` 只允许 `HOLDING`。
- 如果 `remote_busy_ == true`，所有命令都会被拒绝。
- 如果已有 `pending_command_`，新命令会被拒绝。

命令通过检查后会被放入 `pending_command_`，服务立刻返回：

```cpp
response->success = true;
response->message = "received";
```

### 4.2 ready 的静态点计算

机械臂侧 `compute_command_relative_pose()` 现在这样处理：

```cpp
const int task_point_id = static_cast<int>(command.task_index);
if (!nav_pose_tracker_->get_task_point_pose(task_point_id, &target_pose)) {
  ...
}
```

这意味着机械臂侧已经把 `ready.task_index` 正确理解为静态任务点 id。

如果 service 中带了 `x/y`，机械臂会明确忽略：

```cpp
"ignoring service xy=(%.3f, %.3f)."
```

随后使用 `/navigation/state` 和 `/navigation/task_points` 计算机械臂到任务点的相对位置。

### 4.3 ready 的 look_out 执行

`do_ready_sequence()` 将目标点的机械臂相对坐标写入 `look_target`：

```cpp
look_target.position.x = relative_pose.x;
look_target.position.y = relative_pose.y;
```

然后调用：

```cpp
primitives_->do_look_out(look_target)
```

`do_look_out()` 内部将第一轴设置为：

```cpp
goal_q[0] = std::atan2(target.position.y, target.position.x);
```

因此当前 `ready` 的行为就是：根据雷达/导航计算出的任务点相对 yaw，调整机械臂第一轴进入瞭望姿态。

### 4.4 pickup 执行

如果机械臂已经在 `LOOKOUT`，`pickup` 会使用当前瞭望方向：

```cpp
const bool already_lookout = state_ == arm2_task::TaskState::LOOKOUT;
if (!(already_lookout ? sequences_->grasp_from_current_view() : sequences_->grasp_from_perception())) {
  return false;
}
```

`grasp_from_current_view()` 不会重新执行默认前方 `look_out`，只等待关节静止后调用视觉识别。

这与“先用雷达 yaw 对齐，再识别抓取”的目标是匹配的。

### 4.5 arm_event 反馈

pickup 成功后：

```cpp
send_nav_event("grabbed");
...
send_nav_event("completed");
```

place 成功后：

```cpp
send_nav_event("placed");
primitives_->do_reset();
send_nav_event("completed");
```

nav 只接受：

```cpp
event == "grabbed" || event == "placed" || event == "completed"
```

这与机械臂侧当前发送的事件一致。

## 5. 已确认匹配点

- `/arm/mission_event` 的服务类型已经是 `navigation/srv/MissionCommand`。
- `ready.task_index` 当前被机械臂正确当作静态任务点 id 使用。
- `/navigation/task_points` 中存在 `id=1..12`，与机械臂 fallback 参数一致。
- `ready` 不依赖 service 里的 `x/y`，这与 nav 当前实现一致。
- `pickup` 从 `LOOKOUT` 进入时不会覆盖 radar 对齐的第一轴角度。
- 机械臂反馈的 `grabbed/placed/completed` 与 nav 接受的事件集合一致。
- `task_node.launch.py` 默认 `mode=nav`，并可发布 `base_link -> lidar_link` 静态 TF。

## 6. 主要问题与风险

### 6.1 ready 没有 retry，且很容易被机械臂拒绝

nav 的 `pickup/place` 到点请求有 retry，但 `ready` 没有 retry。

`sendReadyForNextMissionTask()` 中如果 arm service 不可用会直接返回：

```cpp
if (context_.arm_mission_client == nullptr || !context_.arm_mission_client->service_is_ready()) {
  context_.status_message = "Arm service unavailable for ready";
  RCLCPP_WARN(logger_, "Skipping arm ready command: arm service is not ready.");
  return;
}
```

并且在收到响应前就设置：

```cpp
completed_task.ready_sent = true;
```

如果机械臂拒绝 ready，nav 只记录 warning，不会再次发送：

```cpp
if (!response->success) {
  context_.status_message = "Arm rejected ready: " + response->message;
  RCLCPP_WARN(logger_, "Arm ready target %d rejected: %s", target_id, response->message.c_str());
  return;
}
```

机械臂侧又会在 `remote_busy_` 时直接拒绝所有 mission command：

```cpp
if (remote_busy_.load()) {
  response->success = false;
  response->message = "arm busy";
  return;
}
```

而机械臂发送 `completed` 时，当前命令线程还没有结束，`remote_busy_` 还没有清除：

```cpp
send_nav_event("completed");
...
remote_busy_.store(false);
```

由于 `task_node` 使用 `MultiThreadedExecutor`，nav 在 `completed` 回调中立刻发送的 `ready` 可以并发进入机械臂服务回调，因此存在真实竞态。

风险结果：

- `ready` 可能在机械臂还 busy 时被拒绝。
- nav 不会重发这个 ready。
- 机械臂可能错过下一次预瞄准。

### 6.2 ready 可能指向 place 目标，但机械臂当前只允许 IDLE/LOOKOUT

nav 的 `ready` 是“下一个 mission 任务”的 preview，不区分下一个任务是 pickup 还是 place。

如果当前刚完成 pickup，下一个任务通常是 place。此时机械臂成功后会进入 `HOLDING`：

```cpp
if (do_grasp_sequence(command)) {
  state_ = arm2_task::TaskState::HOLDING;
  return true;
}
```

但 ready 当前只允许：

```cpp
current_state != arm2_task::TaskState::IDLE &&
current_state != arm2_task::TaskState::LOOKOUT
```

这意味着：

- 如果 ready 在 busy 期间到达，会因为 busy 被拒绝。
- 如果 ready 在 pickup 完全结束后到达，会因为状态是 `HOLDING` 被拒绝。

如果系统设计上不需要对放置点做 ready，这个拒绝不一定阻塞流程，但会导致 nav 记录 `Arm rejected ready`。

如果系统设计上希望对放置点也提前 yaw 对齐，则当前机械臂状态机不支持。

### 6.3 /navigation/state 的坐标语义存在不确定性

机械臂侧 `NavPoseTracker` 当前把 `/navigation/state` 当作 `lidar_pose`：

```cpp
lidar_pose_ = next;
arm_pose_ = arm_pose_from_lidar_pose(next, lidar_in_arm);
robot_pose_ = arm_pose;
```

转换公式是：

```cpp
// map_T_lidar = map_T_arm * arm_T_lidar, so map_T_arm = map_T_lidar * inverse(arm_T_lidar).
arm_pose.yaw = normalize_angle(lidar_pose.yaw - lidar_in_arm.yaw);
arm_pose.x = lidar_pose.x - offset_x_world;
arm_pose.y = lidar_pose.y - offset_y_world;
```

但 nav 发布 `/navigation/state` 时写的是：

```cpp
msg.child_frame_id = "base_link";
```

这带来一个必须实测确认的问题：

- 如果 `/navigation/state` 实际代表雷达位姿，机械臂当前处理是合理的。
- 如果 `/navigation/state` 实际已经代表机体/base 位姿，机械臂当前会重复应用雷达外参，导致 xy/yaw 系统偏差。

当前代码命名和 frame_id 暗示并不完全一致，因此需要在实车上确认 `/navigation/state` 的语义。

### 6.4 pickup/place 日志中的 task_index 解释可能误导

机械臂侧 `log_mission_command()` 对所有 action 都调用：

```cpp
compute_command_relative_pose(command, &relative_pose)
```

而 `compute_command_relative_pose()` 总是把 `command.task_index` 当静态任务点 id：

```cpp
const int task_point_id = static_cast<int>(command.task_index);
```

这对 `ready` 是正确的，但对普通 `pickup/place` 不正确，因为 nav 在普通到点请求中发送的 `task_index` 是 mission 队列序号。

影响：

- 当前执行逻辑主要依赖 `ready` 的相对位姿，实际 pickup/place 不直接使用该计算结果。
- 日志中 `pickup/place` 的 `arm_to_task` 可能失败或指向错误静态点。
- 调试时容易误判导航相对坐标。

## 7. 建议修改方向

### 7.1 arm2_task 侧缓存 ready，避免丢失

由于 nav 是外部包，不建议依赖修改 nav 的 retry 行为。

建议机械臂侧将 `ready` 改成可接收、可延迟执行：

- `ready` 到达时，即使 `remote_busy_ == true`，也先返回 `success=true`。
- 将最新的 `ready` 存入单独的 `pending_ready_command_`。
- 当前 pickup/place 完成后，如果状态允许，再执行缓存的 ready。
- 对连续多个 ready，只保留最后一个即可。

这样可以避免 nav 的一次性 ready 因 busy 竞态丢失。

### 7.2 明确 ready 对 place 的策略

需要确定业务语义：

- 方案 A：`ready` 只用于下一个 pickup 静态箱子点。
- 方案 B：`ready` 同时用于下一个 pickup 和 place 静态点。
- 方案 C：机械臂接受所有 ready，但处于 `HOLDING` 时只缓存不执行，等 place 完成后若目标仍有效再执行。

如果当前需求是“只在抓箱前做雷达 yaw 对齐”，推荐方案 A 或 C。

### 7.3 确认 /navigation/state 的真实 frame 语义

需要用实车或仿真做一个简单校验：

- 让机器人静止，记录 `/navigation/state`。
- 同时记录雷达原始 `/Odometry`。
- 对比 nav 发布的 pose 是否已经转换到机体 `base_link`。
- 如果 `/navigation/state` 已经是 base pose，应将 `task_nav.lidar_extrinsics.enabled` 设为 `false`，或调整机械臂侧命名和处理。
- 如果 `/navigation/state` 是 lidar pose，应保留当前 `base_link -> lidar_link` 逆变换。

### 7.4 限制 pickup/place 日志中的静态点计算

建议只在 `action == "ready"` 时打印 `arm_to_task` 静态任务点日志。

普通 `pickup/place` 到点请求中，日志应明确：

- `task_index` 是 mission 队列序号。
- `point_id` 是路径点 id。
- `x/y` 是导航路径点坐标。

这样可以降低后续联调误判。

## 8. 当前推荐优先级

优先级 1：修改 arm2_task 的 ready 接收与缓存逻辑，避免 nav ready 丢失。

优先级 2：实测确认 `/navigation/state` 是 lidar 位姿还是 base 位姿。

优先级 3：明确 ready 是否需要支持 place 目标。

优先级 4：修正 pickup/place 调试日志，避免把 mission 队列序号误当静态任务点 id。
