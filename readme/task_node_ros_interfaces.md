# task_node ROS2 接口说明

本文档按当前 `src/arm2_task/src/task_node.cpp` 的实现整理，说明 `task_manager_node` 直接创建、订阅、发布或调用的 ROS2 接口，以及这些接口在任务流程中的作用。

> 注意：当前代码已经不是旧版 `/arm/cmd` 话题控制方式。`task_node.cpp` 里没有订阅 `/arm/cmd`，也没有发布 `/arm/status`。远程触发入口是 `/arm/mission_event` 服务，导航回调接口是 `/navigation/arm_event` 服务。

## 1. 节点与启动方式

| 项目 | 当前实现 |
|---|---|
| 节点名 | `task_manager_node` |
| 可执行文件 | `arm2_task/task_node` |
| 默认参数文件 | `src/arm2_task/config/params.yaml` |
| 手动菜单模式 | `task.manual_mode: true` |
| 导航触发模式 | `task.manual_mode: false` |

常用启动方式：

```bash
# 推荐：通过总启动脚本启动 driver、control_node、task_node
bash run_arm.sh

# 单独启动 task_node 时，需要先确保 driver 和 control_node 已运行
ros2 run arm2_task task_node --ros-args --params-file src/arm2_task/config/params.yaml
```

`task_node` 启动后会等待以下条件：

| 条件 | 接口 | 说明 |
|---|---|---|
| 驱动就绪 | `/robot_driver/ready` | 必须收到 `std_msgs/msg/Bool.data=true` |
| 关节状态 | `/arm2/_lowState/joint` | 必须收到至少 5 个有效电机状态 |
| 控制器模式服务 | `set_controller_mode` | 必须可用 |
| 轨迹 action | `move_joint` | 必须可用 |
| 负载估计服务 | `get_payload_estimate` | 仅 `task.require_payload_service=true` 时启动阶段强制等待 |
| 吸盘服务 | `set_suction` | 仅 `task.require_suction_service=true` 时启动阶段强制等待 |

## 2. 话题接口

### 2.1 发布：`/task/target_pose`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> 其他节点 |
| 类型 | `geometry_msgs/msg/Pose` |
| QoS | depth 10，默认可靠 QoS |
| 作用 | 发布当前任务目标点，主要用于调试、可视化、记录 |

发布时机：

| 流程 | 发布内容 |
|---|---|
| 自动抓取 `do_full_grasp` | 抓取目标的 world 位姿 |
| 带朝向抓取 `do_full_grasp_aligned` | 抓取目标的 world 位姿 |
| 自动放置 `do_full_place` | 放置目标的 world 位姿 |
| 3-Phase 抓取/放置 | Phase 2 对齐后的目标位姿 |

使用方法：

```bash
ros2 topic echo /task/target_pose
```

这个话题不是命令入口，外部发布到它不会触发机械臂运动。

### 2.2 订阅：`/arm2/_lowState/joint`

| 项目 | 内容 |
|---|---|
| 方向 | driver/control/sim -> `task_node` |
| 类型 | `robot_msgs/msg/RobotState` |
| QoS | keep last 1，best effort，volatile |
| 作用 | 获取当前 5 个关节的位置 `q` 和速度 `dq` |

消息结构：

```text
robot_msgs/msg/RobotState
  robot_msgs/msg/IMU imu
  robot_msgs/msg/MotorState[] motor_state

robot_msgs/msg/MotorState
  float32 q
  float32 dq
  float32 ddq
  float32 tau_est
  float32 cur
  bool valid
```

`task_node` 只接受 `motor_state.size() >= 5` 且前 5 个 `motor_state[i].valid == true` 的数据。接收后保存：

| 内部变量 | 用途 |
|---|---|
| `q_current_` | 当前 5 轴关节角，单位 rad |
| `dq_current_` | 当前 5 轴关节速度，单位 rad/s |
| `has_robot_data_` | 启动等待条件之一 |

主要使用场景：

| 场景 | 作用 |
|---|---|
| `wait_for_system_ready()` | 确认已有真实关节状态 |
| `wait_joints_still()` | 判断机械臂是否稳定 |
| `phase2_align()` | FK 计算当前末端 XY |
| `do_place_move()` / `do_stack_move_with_orientation()` | 到达目标后基于当前 FK 计算后退点 |
| `phase3_grasp_descend()` / `phase3_place_descend()` | 在当前关节角基础上调整 joint_4 |

调试命令：

```bash
ros2 topic echo /arm2/_lowState/joint
```

### 2.3 订阅：`/robot_driver/ready`

| 项目 | 内容 |
|---|---|
| 方向 | driver/control/sim -> `task_node` |
| 类型 | `std_msgs/msg/Bool` |
| QoS | keep last 1，reliable，transient_local |
| 作用 | 标记底层驱动或仿真器已准备好 |

使用方法：

```bash
ros2 topic echo /robot_driver/ready
```

`transient_local` 会缓存上一条消息，所以排查异常时要确认 driver 是本次启动的实例，而不是读到了旧会话留下的 ready。

## 3. task_node 提供的服务

### 3.1 `/arm/mission_event`

| 项目 | 内容 |
|---|---|
| 方向 | navigation -> `task_node` |
| 类型 | `std_srvs/srv/Trigger` |
| 服务端 | `task_node` |
| 作用 | 导航到达任务点后触发机械臂执行任务 |

服务定义：

```srv
---
bool success
string message
```

当前行为：

| 阶段 | 行为 |
|---|---|
| 收到服务请求 | 立即返回 `success=true, message="received"` |
| `task.manual_mode=false` 且当前状态为 `IDLE` | 异步执行完整抓取序列 |
| 抓取成功后 | 状态切到 `HOLDING` |
| 当前状态为 `HOLDING` 时再次触发 | 当前代码只打印 warning，放置流程尚未接入导航触发 |
| 任务执行中再次触发 | `remote_busy_` 保护，忽略新触发 |

手动测试：

```bash
ros2 service call /arm/mission_event std_srvs/srv/Trigger "{}"
```

当前代码的 `/arm/mission_event` 只有 Trigger，没有 `pickup/place` 字段，因此不能由请求内容指定抓取还是放置。它根据内部 `TaskState` 决定动作，但 `HOLDING` 下的放置逻辑还被注释掉。

> 和 `src/nav/arm_integration/README.md` 的差异：导航侧文档建议 `/arm/mission_event` 使用 `navigation/srv/MissionCommand`，包含 `action=pickup/place`。当前 `task_node.cpp` 实际使用的是 `std_srvs/srv/Trigger`，两者类型不兼容。联调导航时必须统一这一点。

## 4. task_node 调用的服务

### 4.1 `get_pick_pos`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> 抓取感知节点 |
| 类型 | `robot_msgs/srv/GetPickPos` |
| 服务名 | `get_pick_pos`，无 namespace 时实际为 `/get_pick_pos` |
| 作用 | 获取被抓物体的位姿 |

服务定义：

```srv
string object_name
---
bool success
geometry_msgs/PoseStamped pick_pose
```

请求字段：

| 字段 | 来源 | 默认值 | 说明 |
|---|---|---|---|
| `object_name` | `task_step6.pick_object_name` | `"box"` | 传给感知节点的目标名称 |

响应要求：

| 字段 | 要求 |
|---|---|
| `success` | 必须为 `true` |
| `pick_pose.header.frame_id` | 不能为空 |
| TF | 必须能从 `pick_pose.header.frame_id` 查到 `world` |

`task_node` 收到响应后会调用 TF：

```text
lookupTransform("world", pick_pose.header.frame_id, latest, timeout=1s)
```

然后把 `pick_pose.pose` 转换到 `world` 坐标系再用于 IK。

使用场景：

| 场景 | 说明 |
|---|---|
| 菜单 6 Auto Grasp | 先到 `look_out`，再调用 `get_pick_pos` |
| 菜单 12 3-Phase Grasp | Phase 1 和 Phase 2 调用 |
| 菜单 13 3-Phase Place | 当前实现也复用该接口做 scan/align |
| 导航触发抓取 | `do_grasp_sequence()` 中调用 |
| 带朝向抓取 | 会额外采样 3 次取中位数，稳定 roll |

手动测试：

```bash
ros2 service call /get_pick_pos robot_msgs/srv/GetPickPos "{object_name: box}"
```

### 4.2 `get_place_pos`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> 放置感知节点 |
| 类型 | `robot_msgs/srv/GetPlacePos` |
| 服务名 | `get_place_pos`，无 namespace 时实际为 `/get_place_pos` |
| 作用 | 获取放置框或目标 frame 的位姿 |

服务定义：

```srv
string frame_name
---
bool success
geometry_msgs/PoseStamped place_pose
```

请求字段：

| 字段 | 来源 | 默认值 | 说明 |
|---|---|---|---|
| `frame_name` | `task_place_frame.frame_name` | `"target_frame"` | 放置目标名称 |

响应要求和 `get_pick_pos` 类似：`success=true`、`place_pose.header.frame_id` 非空，且存在 `frame_id -> world` 的 TF。

使用场景：

| 场景 | 说明 |
|---|---|
| 菜单 4 Auto Place | 狗头相机感知目标框，然后执行 `pre-place -> place -> suction OFF -> retreat` |
| 导航放置流程 | `do_place_sequence()` 已实现，但当前没有从 `/arm/mission_event` 接入 |

手动测试：

```bash
ros2 service call /get_place_pos robot_msgs/srv/GetPlacePos "{frame_name: target_frame}"
```

### 4.3 `get_stack_pos`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> 叠放感知节点 |
| 类型 | `robot_msgs/srv/GetPlacePos` |
| 服务名 | `get_stack_pos`，无 namespace 时实际为 `/get_stack_pos` |
| 作用 | 获取目标箱子上表面位姿，用于叠放 |

服务定义同 `GetPlacePos`：

```srv
string frame_name
---
bool success
geometry_msgs/PoseStamped place_pose
```

使用场景：

| 场景 | 说明 |
|---|---|
| 菜单 14 Auto Stack | 获取箱子上表面位姿，执行 `pre-stack -> stack -> suction OFF -> retreat` |
| 菜单 15 Manual Stack | 不调用服务，直接用手动输入 |

重要细节：

当前代码中 service client 固定创建为：

```cpp
stack_client_ = this->create_client<robot_msgs::srv::GetPlacePos>("get_stack_pos");
```

参数 `task_stack.stack_service` 的名字看起来像服务名，但当前实际被传进了 `request.frame_name`：

```cpp
call_stack_service_sync(stack_service_name_, &box_top_pose);
request->frame_name = frame_name;
```

也就是说，当前要改 `get_stack_pos` 的服务名需要改代码；只改 `task_stack.stack_service` 不会改变 service client 的名字。

手动测试：

```bash
ros2 service call /get_stack_pos robot_msgs/srv/GetPlacePos "{frame_name: get_stack_pos}"
```

如果你的叠放感知节点期望的 frame 名不是 `get_stack_pos`，需要把请求里的 `frame_name` 换成对应名称，或者修正 `task_stack.stack_service` 这个参数的命名/用途。

### 4.4 `set_suction`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> 吸盘节点 |
| 类型 | `robot_msgs/srv/SetSuction` |
| 服务名 | `set_suction`，无 namespace 时实际为 `/set_suction` |
| 作用 | 打开或关闭吸盘 |

服务定义：

```srv
bool activate
---
bool success
```

请求含义：

| `activate` | 作用 |
|---|---|
| `true` | 吸盘 ON |
| `false` | 吸盘 OFF |

可选/必选行为：

| 参数 | 行为 |
|---|---|
| `task.require_suction_service=false` | 服务不可用时跳过吸盘命令，不把任务视为失败 |
| `task.require_suction_service=true` | 启动阶段等待服务；调用时不可用或失败会导致任务失败 |

使用场景：

| 函数/菜单 | 行为 |
|---|---|
| `do_suction_on()` | 等待机械臂稳定 -> `set_suction(true)` -> 切 `loaded` 模式 |
| `do_suction_off()` | `set_suction(false)` -> 切 `moving` 模式 |
| 菜单 1 Reset | 先关闭吸盘 |
| 菜单 4/13/14/15 Place/Stack | 到达目标后关闭吸盘 |
| 菜单 8 Release | 关闭吸盘 |

手动测试：

```bash
ros2 service call /set_suction robot_msgs/srv/SetSuction "{activate: true}"
ros2 service call /set_suction robot_msgs/srv/SetSuction "{activate: false}"
```

### 4.5 `set_controller_mode`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> `control_node` |
| 类型 | `robot_msgs/srv/SetControllerMode` |
| 服务名 | `set_controller_mode`，无 namespace 时实际为 `/set_controller_mode` |
| 作用 | 切换控制器增益/补偿模式 |

服务定义：

```srv
string mode
---
bool success
string message
```

当前 `task_node` 主要使用的模式：

| mode | 使用场景 |
|---|---|
| `moving` | 普通运动、复位、抓取前、放置前、关闭吸盘后 |
| `loaded` | 吸盘吸住物体后、carry 姿态后 |
| `idle` | `do_reset_suction()` 中使用 |

`control_node` 还支持哪些模式取决于参数里的 `gains.*` 配置，例如 `gravity_comp`、`teach_pendant`、`teach_drag` 等，但 `task_node` 当前没有在主流程里主动切这些模式。

手动测试：

```bash
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: moving}"
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: loaded}"
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: idle}"
```

### 4.6 `get_payload_estimate`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> `control_node` |
| 类型 | `robot_msgs/srv/GetPayloadEstimate` |
| 服务名 | `get_payload_estimate`，无 namespace 时实际为 `/get_payload_estimate` |
| 作用 | 请求控制器估计当前负载质量 |

服务定义：

```srv
---
float32 mass
bool success
string message
```

使用场景：

| 场景 | 说明 |
|---|---|
| 菜单 11 Estimate payload | 调用服务并打印 `mass` |
| `task.require_payload_service=true` | 启动阶段会等待该服务可用 |

手动测试：

```bash
ros2 service call /get_payload_estimate robot_msgs/srv/GetPayloadEstimate "{}"
```

### 4.7 `set_payload_state`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> `control_node` |
| 类型 | `robot_msgs/srv/SetPayloadState` |
| 默认服务名 | `set_payload_state`，无 namespace 时实际为 `/set_payload_state` |
| 服务名参数 | `task.payload_service` |
| 作用 | 通知控制器当前是否带负载，以及负载质量/质心 |

服务定义：

```srv
bool has_load
float64 mass
float64[3] com
---
bool success
string message
```

请求字段来源：

| 字段 | 置入逻辑 | 参数来源 |
|---|---|---|
| `has_load` | 启用时为 `task.payload_default.has_load`，清除时强制 `false` | `task.payload_default.has_load` |
| `mass` | 固定填默认负载质量 | `task.payload_default.mass` |
| `com` | 固定填默认质心 | `task.payload_default.com` |

使用场景：

| 菜单 | 行为 |
|---|---|
| 16 Enable payload model | `request_payload_state(true)` |
| 17 Clear payload model | `request_payload_state(false)` |

注意：当前自动抓取/放置流程主要通过 `set_controller_mode("loaded")` 切高增益，没有自动调用 `set_payload_state`。要启用动力学负载模型，需要手动菜单 16，或者后续把该调用接入抓取成功后。

手动测试：

```bash
ros2 service call /set_payload_state robot_msgs/srv/SetPayloadState \
  "{has_load: true, mass: 0.5, com: [0.0, 0.0, 0.2219]}"

ros2 service call /set_payload_state robot_msgs/srv/SetPayloadState \
  "{has_load: false, mass: 0.5, com: [0.0, 0.0, 0.2219]}"
```

### 4.8 `/navigation/arm_event`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> navigation |
| 类型 | `navigation/srv/StringCommand` |
| 服务名 | `/navigation/arm_event` |
| 作用 | 机械臂向导航汇报任务阶段事件 |

服务定义：

```srv
string message
---
bool success
string message
```

当前 `task_node` 会发送的事件：

| 事件 | 发送时机 |
|---|---|
| `grabbed` | 导航触发抓取流程中，吸盘 ON 后等待 0.5s |
| `completed` | 导航触发抓取流程中，机械臂移动到 `carry` 姿态并切到 `loaded` 后 |

当前没有发送的事件：

| 事件 | 状态 |
|---|---|
| `placed` | 导航文档里有定义，但当前 `task_node.cpp` 放置流程还没有接入导航触发 |

手动测试导航侧：

```bash
ros2 service call /navigation/arm_event navigation/srv/StringCommand "{message: grabbed}"
ros2 service call /navigation/arm_event navigation/srv/StringCommand "{message: completed}"
```

## 5. Action 接口

### 5.1 `move_joint`

| 项目 | 内容 |
|---|---|
| 方向 | `task_node` -> `control_node` |
| 类型 | `robot_msgs/action/MoveJoint` |
| action 名 | `move_joint`，无 namespace 时实际为 `/move_joint` |
| 作用 | 发送 5 轴关节目标，执行单点 PTP 或多路点 blending 轨迹 |

Action 定义：

```action
# Goal
float64[] joint_targets
int32 num_points
float64 max_velocity
float64 max_acceleration
float64 blend_radius
---
# Result
bool success
string message
---
# Feedback
float64 progress
float64[] current_errors
```

`task_node` 发送 goal 的方式：

| Goal 字段 | 来源/含义 |
|---|---|
| `joint_targets` | 5 轴关节目标按 waypoint 展平成一维数组，单位 rad |
| `num_points` | waypoint 数量 |
| `max_velocity` | `trajectory_planner.max_velocity` |
| `max_acceleration` | `trajectory_planner.max_acceleration` |
| `blend_radius` | `trajectory_planner.dist_threshold` |

例如两个 waypoint 会按下面顺序展平：

```text
[q0_p1, q1_p1, q2_p1, q3_p1, q4_p1,
 q0_p2, q1_p2, q2_p2, q3_p2, q4_p2]
```

等待与失败处理：

| 阶段 | 超时/判断 |
|---|---|
| 等 action server | 每次发送前最多等 10s |
| 等 action result | 默认最多等 30s |
| result 判断 | ROS action result code 必须是 `SUCCEEDED`，且 `result.success=true` |
| feedback | 当前代码没有使用 feedback |

手动测试：

```bash
ros2 action send_goal /move_joint robot_msgs/action/MoveJoint \
  "{joint_targets: [0.0, 1.57, -1.57, 0.0, 0.0], num_points: 1, max_velocity: 1.0, max_acceleration: 2.0, blend_radius: 0.05}" \
  --feedback
```

## 6. TF 依赖

`task_node` 自己创建 `tf2_ros::Buffer` 和 `tf2_ros::TransformListener`。感知服务返回的 `PoseStamped` 不直接使用，而是转换到 `world` 后再进入 IK。

| 感知接口 | 常见返回 frame | 需要存在的 TF |
|---|---|---|
| `get_pick_pos` | `camera_link` | `camera_link -> world` 可查 |
| `get_place_pos` | `dog_camera_link` | `dog_camera_link -> world` 可查 |
| `get_stack_pos` | `dog_camera_link` | `dog_camera_link -> world` 可查 |

实际查找方式是：

```text
lookupTransform("world", response_frame, latest, timeout=1s)
```

调试命令：

```bash
ros2 run tf2_ros tf2_echo world camera_link
ros2 run tf2_ros tf2_echo world dog_camera_link
```

如果感知服务返回成功但 `task_node` 报 TF2 error，优先检查：

| 检查项 | 说明 |
|---|---|
| `header.frame_id` | 不能为空，且要和 TF 树里的 frame 名一致 |
| 静态外参 | `control_node` 是否已发布相机静态 TF |
| 坐标方向 | 感知节点返回的是相机系还是已经转过的 world 系 |

## 7. 关键参数

这些参数都从传入 `task_node` 的参数文件读取。当前 `run_arm.sh` 默认只加载 `src/arm2_task/config/params.yaml`。

### 7.1 启动与服务必选项

| 参数 | 默认值 | 作用 |
|---|---|---|
| `task.manual_mode` | 当前 `params.yaml` 为 `true` | `true` 进入终端菜单；`false` 等待 `/arm/mission_event` |
| `task.require_payload_service` | `false` | 是否把负载相关服务作为强依赖 |
| `task.require_suction_service` | `false` | 是否把吸盘服务作为强依赖 |
| `task.payload_service` | `set_payload_state` | `SetPayloadState` 服务名 |

### 7.2 轨迹参数

| 参数 | 作用 |
|---|---|
| `trajectory_planner.max_velocity` | 发送 `move_joint` goal 时的最大速度 |
| `trajectory_planner.max_acceleration` | 发送 `move_joint` goal 时的最大加速度 |
| `trajectory_planner.dist_threshold` | 发送 `move_joint` goal 时的 `blend_radius` |

注意：`task_node` 当前没有读取 `trajectory_planner.action_name`、`state_topic`、`ready_topic`。这些名字在代码里是硬编码的。

### 7.3 预设关节姿态

| 参数 | 作用 |
|---|---|
| `presets.reset` | 复位/收回姿态，角度制 |
| `presets.look_out` | 瞭望姿态，角度制 |
| `presets.load` | 俯瞰姿态，角度制 |
| `presets.carry` | 携带箱子收回姿态，角度制 |

读取后会从角度转换成 rad，再发送给 `move_joint`。

### 7.4 抓取参数

| 参数 | 作用 |
|---|---|
| `task_step6.pick_object_name` | `get_pick_pos` 请求里的 `object_name` |
| `task_step6.grasp_pitch` | 抓取/放置时末端 pitch 基准 |
| `task_step6.tool_pitch_offset` | 实机 pitch 标定偏移 |
| `task_step6.tool_yaw_offset` | joint_0 yaw 额外偏移 |
| `task_step6.object_height` | 物体高度补偿 |
| `task_step6.pre_grasp_offset` | 预抓取点相对抓取点的 Z 向高度 |
| `task_step6.tool_offset_x/y/z` | 吸盘目标点补偿 |
| `task_step6.tool_tip_length` | 吸盘接触面到 Link_5 origin 的长度补偿 |
| `task_step6.use_mock_target` | 菜单 6 是否使用 mock 目标 |
| `task_step6.mock_x/y/z` | mock 抓取目标 |

### 7.5 放置与叠放参数

| 参数 | 作用 |
|---|---|
| `task_place.pre_place_offset` | 普通放置预放置高度 |
| `task_place.retreat_offset` | 放置/叠放后垂直后退距离 |
| `task_place_frame.frame_name` | `get_place_pos` 请求里的 `frame_name` |
| `task_place_frame.hover_height` | 自动放置预放置高度 |
| `task_place_frame.contact_offset` | 放置接触点 Z 偏移 |
| `task_place_frame.use_mock_target` | 菜单 4 是否使用 mock 放置目标 |
| `task_place_frame.mock_x/y/z/yaw` | mock 放置目标 |
| `task_stack.stack_service` | 当前代码实际作为 `get_stack_pos` 请求里的 `frame_name` 使用 |
| `task_stack.hover_height` | 叠放预放置高度 |
| `task_stack.contact_offset` | 叠放接触点 Z 偏移 |
| `task_stack.use_mock_target` | 菜单 14 是否使用 mock 叠放目标 |
| `task_stack.mock_x/y/z/yaw` | mock 叠放目标 |

### 7.6 视觉对齐参数

| 参数 | 作用 |
|---|---|
| `visual_align.align_threshold` | Phase 2 中末端 XY 与目标 XY 的收敛阈值 |
| `visual_align.max_iters` | Phase 2 最大迭代次数 |

## 8. 手动菜单与接口调用关系

`task.manual_mode=true` 时，`task_node` 在终端显示菜单。各菜单项和 ROS 接口关系如下：

| 菜单 | 功能 | 主要 ROS 接口 |
|---|---|---|
| 1 | Reset | `set_suction(false)`、`set_controller_mode(moving)`、`move_joint(reset)` |
| 2 | Joint preset A | `set_controller_mode(moving)`、`move_joint` |
| 3 | Joint preset B | `set_controller_mode(moving)`、`move_joint` |
| 4 | Auto Place | `get_place_pos`、TF、`move_joint`、`set_suction(false)` |
| 5 | Manual Place | 手动输入目标，`move_joint`、`set_suction(false)` |
| 6 | Auto Grasp | `get_pick_pos`、TF、`/task/target_pose`、`move_joint`、`set_suction(true)` |
| 7 | Manual Grasp | 手动输入目标，`/task/target_pose`、`move_joint`、`set_suction(true)` |
| 8 | Release | `set_suction(false)`、`set_controller_mode(moving)` |
| 9 | Carry reset | `set_controller_mode(moving)`、`move_joint(carry)`、`set_controller_mode(loaded)` |
| 10 | Move to load preset | `set_controller_mode(moving)`、`move_joint(load)` |
| 11 | Estimate payload | `get_payload_estimate` |
| 12 | 3-Phase Grasp | `get_pick_pos`、TF、`move_joint`、`set_suction(true)` |
| 13 | 3-Phase Place | `get_pick_pos`、TF、`move_joint`、`set_suction(false)` |
| 14 | Auto Stack | `get_stack_pos`、TF、`move_joint`、`set_suction(false)` |
| 15 | Manual Stack | 手动输入目标，`move_joint`、`set_suction(false)` |
| 16 | Enable payload model | `set_payload_state(has_load=true)` |
| 17 | Clear payload model | `set_payload_state(has_load=false)` |
| 0 | Exit | `rclcpp::shutdown()` |

## 9. 导航触发模式流程

`task.manual_mode=false` 时，`task_node` 不显示菜单，而是进入 `run_remote_control()`，等待 `/arm/mission_event`。

当前抓取流程：

```text
navigation calls /arm/mission_event
  -> task_node immediately replies success=true
  -> set_controller_mode("moving")
  -> move_joint(look_out toward front)
  -> get_pick_pos(object_name)
  -> TF to world
  -> publish /task/target_pose
  -> set_controller_mode("moving")
  -> move_joint(look_out toward target)
  -> get_pick_pos repeated samples for aligned roll
  -> move_joint(pre_grasp, grasp)
  -> set_suction(true)
  -> set_controller_mode("loaded")
  -> call /navigation/arm_event with "grabbed"
  -> set_controller_mode("moving")
  -> move_joint(carry)
  -> set_controller_mode("loaded")
  -> call /navigation/arm_event with "completed"
  -> internal state becomes HOLDING
```

当前放置流程状态：

| 项目 | 当前实现 |
|---|---|
| `do_place_sequence()` | 函数已写好，会调用 `get_place_pos` 并执行放置 |
| `/arm/mission_event` 第二次触发 | 在 `HOLDING` 状态下只打印 warning，实际放置调用被注释 |
| 导航事件 | 放置完成后的 `placed/completed` 发送也还被注释 |

因此，当前导航联调只算完整接上了“到点触发抓取 -> 抓取完成回报”。放置任务还需要把 `/arm/mission_event` 的请求类型和状态机补齐。

## 10. 当前没有实现的旧接口

仓库里有旧文档提到下面接口，但当前 `task_node.cpp` 没有创建这些 publisher/subscriber：

| 旧接口 | 当前状态 |
|---|---|
| `/arm/cmd` `std_msgs/msg/String` | 未订阅 |
| `/arm/status` `std_msgs/msg/String` | 未发布 |
| `task.remote_mode` 参数 | 未读取，当前读取的是 `task.manual_mode` |

如果外部系统还按旧文档用 `/arm/cmd` 发送 `"grasp"`/`"place"`，当前 `task_node` 不会响应。

## 11. 接口排查清单

启动前建议检查：

```bash
ros2 topic echo /robot_driver/ready --once
ros2 topic echo /arm2/_lowState/joint --once
ros2 service list | grep -E 'set_controller_mode|get_pick_pos|get_place_pos|get_stack_pos|set_suction|get_payload_estimate|set_payload_state|arm_event|mission_event'
ros2 action list | grep move_joint
```

抓取感知排查：

```bash
ros2 service call /get_pick_pos robot_msgs/srv/GetPickPos "{object_name: box}"
ros2 run tf2_ros tf2_echo world camera_link
```

放置/叠放感知排查：

```bash
ros2 service call /get_place_pos robot_msgs/srv/GetPlacePos "{frame_name: target_frame}"
ros2 service call /get_stack_pos robot_msgs/srv/GetPlacePos "{frame_name: get_stack_pos}"
ros2 run tf2_ros tf2_echo world dog_camera_link
```

导航触发排查：

```bash
ros2 service call /arm/mission_event std_srvs/srv/Trigger "{}"
ros2 service call /navigation/arm_event navigation/srv/StringCommand "{message: grabbed}"
ros2 service call /navigation/arm_event navigation/srv/StringCommand "{message: completed}"
```

动作链路排查：

```bash
ros2 action info /move_joint
ros2 action send_goal /move_joint robot_msgs/action/MoveJoint \
  "{joint_targets: [0.0, 1.57, -1.57, 0.0, 0.0], num_points: 1, max_velocity: 1.0, max_acceleration: 2.0, blend_radius: 0.05}" \
  --feedback
```
