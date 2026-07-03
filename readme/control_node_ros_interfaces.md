# control_node ROS2 接口说明

本文档按当前 `src/arm2_task/src/control_node.cpp` 的实现整理，说明 `controller_node` 直接创建、订阅、发布或提供的 ROS2 接口，以及这些接口的使用方式与作用。

> 注意：本文只描述当前 `control_node.cpp` 实际使用的接口。`src/arm2_task/config/params.yaml` 和 `control_params.yaml` 中有一些历史/备用参数并未被当前 `control_node` 读取，文末已单独标出。

## 1. 节点与启动方式

| 项目 | 当前实现 |
|---|---|
| 节点名 | `controller_node` |
| 可执行文件 | `arm2_task/control_node` |
| 源码文件 | `src/arm2_task/src/control_node.cpp` |
| 默认参数文件 | `src/arm2_task/config/params.yaml` 或 launch 默认的 install/share 参数 |
| 控制周期 | 10 ms，约 100 Hz |
| 关节数量 | 固定按 5 轴处理 |

常用启动方式：

```bash
# 推荐：通过总启动脚本启动 driver、control_node、task_node
bash run_arm.sh

# 单独启动 control_node，使用源码参数文件
ros2 run arm2_task control_node --ros-args --params-file src/arm2_task/config/params.yaml

# 使用 launch 启动
ros2 launch arm2_task control_node.launch.py

# launch 时显式指定源码参数文件，避免使用 install/share 中的旧参数
ros2 launch arm2_task control_node.launch.py \
  params_path:=$PWD/src/arm2_task/config/params.yaml
```

`control_node` 启动后不会立即输出有效控制，必须同时满足：

| 条件 | 接口 | 说明 |
|---|---|---|
| 驱动 ready | `/robot_driver/ready` | 收到 `std_msgs/msg/Bool.data=true` |
| 有效关节状态 | `/arm2/_lowState/joint` | 收到至少 5 个 `valid=true` 的电机状态 |

满足后控制循环开始发布 `/arm2/_lowCmd/command`。

## 2. 话题接口

### 2.1 订阅：`/arm2/_lowState/joint`

| 项目 | 内容 |
|---|---|
| 方向 | driver/sim -> `control_node` |
| 类型 | `robot_msgs/msg/RobotState` |
| QoS | keep last 1，best effort，volatile |
| 作用 | 获取当前 5 个关节的反馈位置、速度和估计力矩 |

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

`control_node` 只接受：

| 检查项 | 要求 |
|---|---|
| `motor_state.size()` | 必须 >= 5 |
| `motor_state[0..4].valid` | 必须全部为 `true` |

接收后保存：

| 字段 | 内部用途 |
|---|---|
| `q` | `current_q_`，当前关节角，单位 rad |
| `dq` | `current_dq_`，当前关节速度，单位 rad/s |
| `tau_est` | `current_tau_`，用于负载估计 |

同时每次收到有效状态后，会基于当前关节角计算正运动学并发布 `world -> Link_4` 动态 TF。

调试命令：

```bash
ros2 topic echo /arm2/_lowState/joint
ros2 topic hz /arm2/_lowState/joint
```

### 2.2 订阅：`/robot_driver/ready`

| 项目 | 内容 |
|---|---|
| 方向 | driver/sim -> `control_node` |
| 类型 | `std_msgs/msg/Bool` |
| QoS | keep last 1，reliable，transient local |
| 作用 | 表示底层电机驱动是否可用 |

`data=true` 时，且已经收到有效关节状态后，`control_node` 才会接受 action 目标并发布控制命令。

调试命令：

```bash
ros2 topic echo /robot_driver/ready
```

### 2.3 发布：`/arm2/_lowCmd/command`

| 项目 | 内容 |
|---|---|
| 方向 | `control_node` -> driver/sim |
| 类型 | `robot_msgs/msg/RobotCommand` |
| QoS | keep last 1，best effort，volatile |
| 发布频率 | 控制循环约 100 Hz |
| 作用 | 向底层驱动发送 5 轴位置、速度、力矩前馈和 PD 增益 |

消息结构：

```text
robot_msgs/msg/RobotCommand
  robot_msgs/msg/MotorCommand[] motor_command

robot_msgs/msg/MotorCommand
  float32 q
  float32 dq
  float32 tau
  float32 kp
  float32 kd
```

每个周期发布 5 个 `MotorCommand`：

| 字段 | 来源 | 作用 |
|---|---|---|
| `q` | 轨迹插值结果或锁定目标 `command_q_` | 目标关节角，单位 rad |
| `dq` | 轨迹插值结果 | 目标关节速度，单位 rad/s |
| `tau` | `DynamicsManager::getFeedForwardTorque()` | 逆动力学 + 摩擦补偿前馈 |
| `kp` | 当前模式的 `gains.<mode>.kp[i]` | 位置刚度 |
| `kd` | 当前模式的 `gains.<mode>.kd[i]` | 速度阻尼 |

底层电机通常按类似下面的组合产生输出：

```text
tau_output ~= kp * (q_cmd - q_feedback)
            + kd * (dq_cmd - dq_feedback)
            + tau_feedforward
```

调试命令：

```bash
ros2 topic echo /arm2/_lowCmd/command
ros2 topic hz /arm2/_lowCmd/command
```

### 2.4 发布：`/debug/friction_torque`

| 项目 | 内容 |
|---|---|
| 方向 | `control_node` -> 调试工具 |
| 类型 | `std_msgs/msg/Float32MultiArray` |
| QoS | depth 10，默认 QoS |
| 发布频率 | 控制循环约 100 Hz |
| 作用 | 发布当前期望速度下计算出的 5 轴摩擦补偿力矩 |

摩擦模型：

```text
tau_f = fc * tanh(alpha * dq) + fv * dq * GearRatio^2
```

其中参数来自：

```yaml
dynamics:
  friction:
    GearRatio: [...]
    alpha: ...
    fc: [...]
    fv: [...]
```

注意：这个话题只发布摩擦项，不包含完整逆动力学前馈。完整前馈会写入 `/arm2/_lowCmd/command` 中每个 `motor_command[i].tau`。

调试命令：

```bash
ros2 topic echo /debug/friction_torque
```

### 2.5 发布：`/tf`

| 项目 | 内容 |
|---|---|
| 方向 | `control_node` -> TF 系统 |
| 类型 | `tf2_msgs/msg/TFMessage` |
| 作用 | 发布机械臂当前末端相关动态 TF |

当前动态 TF：

| parent | child | 来源 |
|---|---|---|
| `world` | `Link_4` | 使用当前 `current_q_` 做 FK 计算 |

发布时机：每次收到有效 `/arm2/_lowState/joint` 后发布一次。

调试命令：

```bash
ros2 run tf2_ros tf2_echo world Link_4
```

### 2.6 发布：`/tf_static`

| 项目 | 内容 |
|---|---|
| 方向 | `control_node` -> TF 系统 |
| 类型 | `tf2_msgs/msg/TFMessage` |
| 作用 | 发布相机外参静态 TF |

当前静态 TF：

| parent | child | 参数来源 |
|---|---|---|
| `camera_extrinsics.parent_frame` | `camera_extrinsics.child_frame` | `camera_extrinsics.pos/quat` |
| `dog_camera_extrinsics.parent_frame` | `dog_camera_extrinsics.child_frame` | `dog_camera_extrinsics.pos/quat` |

默认/常用配置：

| TF | 当前配置含义 |
|---|---|
| `Link_4 -> camera_link` | 机械臂末端相机外参 |
| `world -> dog_camera_link` | 狗头相机相对机械臂基座/world 的外参 |

调试命令：

```bash
ros2 run tf2_ros tf2_echo Link_4 camera_link
ros2 run tf2_ros tf2_echo world dog_camera_link
```

## 3. 服务接口

### 3.1 提供：`/set_controller_mode`

| 项目 | 内容 |
|---|---|
| 方向 | 外部节点 -> `control_node` |
| 类型 | `robot_msgs/srv/SetControllerMode` |
| 作用 | 切换当前 PD 参数组 |

服务定义：

```text
string mode
---
bool success
string message
```

当前代码实际加载的模式固定为：

| 模式 | 参数来源 | 典型用途 |
|---|---|---|
| `idle` | `gains.idle.kp/kd` | 低刚度待机 |
| `gravity_comp` | `gains.gravity_comp.kp/kd` | 低 PD + 动力学前馈 |
| `moving` | `gains.moving.kp/kd` | 普通运动 |
| `loaded` | `gains.loaded.kp/kd` | 负载/高刚度运动 |

调用条件：

| 条件 | 结果 |
|---|---|
| driver 未 ready 或无有效状态 | 返回 `success=false` |
| `mode` 不在上述 4 个模式中 | 返回 `success=false` |
| 模式有效且系统 ready | 切换 `current_gains_` 并返回 `success=true` |

使用方法：

```bash
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: moving}"
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: gravity_comp}"
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: loaded}"
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: idle}"
```

注意：`control_params.yaml` 中虽然有 `teach_pendant`、`teach_drag`，但当前 `control_node.cpp` 的 `load_all_gains()` 只加载 `idle/gravity_comp/moving/loaded`。直接调用 `teach_pendant` 或 `teach_drag` 会返回未知模式。

### 3.2 提供：`/get_payload_estimate`

| 项目 | 内容 |
|---|---|
| 方向 | 外部节点 -> `control_node` |
| 类型 | `robot_msgs/srv/GetPayloadEstimate` |
| 作用 | 根据当前反馈力矩估计末端负载质量 |

服务定义：

```text
---
float32 mass
bool success
string message
```

内部计算使用：

| 数据 | 来源 |
|---|---|
| 当前关节角 `q` | `/arm2/_lowState/joint` |
| 当前关节速度 `dq` | `/arm2/_lowState/joint` |
| 当前估计力矩 `tau_est` | `/arm2/_lowState/joint` |
| 空载动力学模型 | `DynamicsManager` + URDF |
| 摩擦模型 | `dynamics.friction.*` |

调用条件：

| 条件 | 结果 |
|---|---|
| driver 未 ready 或无有效状态 | 返回 `success=false` |
| 系统 ready | 返回估计质量 `mass` |

使用方法：

```bash
ros2 service call /get_payload_estimate robot_msgs/srv/GetPayloadEstimate "{}"
```

### 3.3 提供：`/set_payload_state`

| 项目 | 内容 |
|---|---|
| 方向 | 外部节点 -> `control_node` |
| 类型 | `robot_msgs/srv/SetPayloadState` |
| 默认服务名 | `set_payload_state` |
| 可配置参数 | `inverse_dynamics.payload_service` |
| 作用 | 告诉动力学模型当前是否带载，以及负载质量和质心 |

服务定义：

```text
bool has_load
float64 mass
float64[3] com
---
bool success
string message
```

字段含义：

| 字段 | 说明 |
|---|---|
| `has_load` | `true` 表示启用负载模型，`false` 表示清除负载模型 |
| `mass` | 负载质量，单位 kg，必须为非负有限值 |
| `com` | 负载质心，三维向量，必须全部为有限值 |

使用方法：

```bash
# 启用 0.5 kg 负载模型
ros2 service call /set_payload_state robot_msgs/srv/SetPayloadState \
  "{has_load: true, mass: 0.5, com: [0.0, 0.0, 0.2219]}"

# 清除负载模型
ros2 service call /set_payload_state robot_msgs/srv/SetPayloadState \
  "{has_load: false, mass: 0.0, com: [0.0, 0.0, 0.0]}"
```

注意：如果参数 `inverse_dynamics.payload_service` 改成其他名字，服务名也会随之改变。

## 4. Action 接口

### 4.1 提供：`/move_joint`

| 项目 | 内容 |
|---|---|
| 方向 | 外部节点 -> `control_node` |
| 类型 | `robot_msgs/action/MoveJoint` |
| 作用 | 发送 5 轴关节空间目标，支持单点 PTP 和多点 blending |

Action 定义：

```text
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

Goal 字段说明：

| 字段 | 说明 |
|---|---|
| `joint_targets` | 展平后的目标数组，必须按 `[p0_j0, p0_j1, ..., p0_j4, p1_j0, ...]` 排列 |
| `num_points` | 目标点数量；每个点固定 5 个关节 |
| `max_velocity` | 本次运动最大速度；`<=0` 时使用节点默认值 |
| `max_acceleration` | 本次运动最大加速度；`<=0` 时使用节点默认值 |
| `blend_radius` | 多点轨迹混合半径；`<=0` 时使用默认 `0.15` |

行为：

| 场景 | 行为 |
|---|---|
| driver 未 ready 或无有效状态 | goal 被拒绝 |
| 只有 1 个点 | 从当前关节角规划到目标点 |
| 多个点 | 依次进入队列，接近当前目标时预规划下一段并 blending |
| 新 goal 到来 | 清空旧队列和当前段，重新按新 goal 执行 |
| cancel | 清空队列，停止当前运动，返回 `success=false` |

发送单点目标示例：

```bash
ros2 action send_goal /move_joint robot_msgs/action/MoveJoint \
  "{joint_targets: [0.0, 1.57, -1.57, 0.0, 0.0], num_points: 1, max_velocity: 0.6, max_acceleration: 1.0, blend_radius: 0.0}" \
  --feedback
```

发送两个目标点示例：

```bash
ros2 action send_goal /move_joint robot_msgs/action/MoveJoint \
  "{joint_targets: [0.0, 1.57, -1.57, 0.0, 0.0, 0.3, 1.3, -1.2, 0.2, 0.0], num_points: 2, max_velocity: 0.8, max_acceleration: 1.2, blend_radius: 0.15}" \
  --feedback
```

单位约定：

| 数据 | 单位 |
|---|---|
| `joint_targets` | rad |
| `max_velocity` | rad/s |
| `max_acceleration` | rad/s^2 |
| `blend_radius` | 关节空间距离，rad 量级 |

注意：当前 action 的 `Feedback.current_errors` 定义存在，但 `control_node.cpp` 目前只填充 `progress`，没有填充 `current_errors`。

## 5. 参数接口

### 5.1 启动与模型参数

| 参数 | 类型 | 默认值 | 作用 |
|---|---|---|---|
| `urdf_path` | string | `urdf/arm2.urdf` | 相对 `arm2_task` share 目录的 URDF 路径 |
| `robot_geometry.l1` | double | `0.0845` | 几何参数，传给 `KinematicsEngine` |
| `robot_geometry.l2` | double | `0.350005` | 几何参数，传给 `KinematicsEngine` |
| `robot_geometry.l3` | double | `0.243441` | 几何参数，传给 `KinematicsEngine` |
| `robot_geometry.l4` | double | `0.046` | 几何参数，传给 `KinematicsEngine` |

### 5.2 轨迹默认限制参数

| 参数 | 类型 | 默认值 | 作用 |
|---|---|---|---|
| `max_joint_velocity` | double | `1.2` | action goal 的 `max_velocity <= 0` 时使用 |
| `max_acceleration_limit` | double | `1.0` | action goal 的 `max_acceleration <= 0` 时使用 |

注意：当前 `control_node.cpp` 不读取 `trajectory_planner.max_velocity`、`trajectory_planner.max_acceleration`。如果 action goal 里填 `0`，实际 fallback 是上表两个参数，而不是 `trajectory_planner.*`。

### 5.3 摩擦与动力学前馈参数

| 参数 | 类型 | 作用 |
|---|---|---|
| `dynamics.friction.fc` | double[5] | 库仑摩擦系数 |
| `dynamics.friction.fv` | double[5] | 粘性摩擦系数 |
| `dynamics.friction.GearRatio` | double[5] | 传动比，用于粘性摩擦平方项 |
| `dynamics.friction.alpha` | double | `tanh(alpha * dq)` 平滑系数 |

完整前馈计算：

```text
tau_ff = inverse_dynamics(q_des, dq_des, ddq_des) + friction(dq_des)
```

`tau_ff` 会写入 `/arm2/_lowCmd/command.motor_command[i].tau`。

### 5.4 增益模式参数

| 参数 | 类型 | 作用 |
|---|---|---|
| `gains.idle.kp` | double[5] | idle 模式位置增益 |
| `gains.idle.kd` | double[5] | idle 模式速度增益 |
| `gains.gravity_comp.kp` | double[5] | gravity_comp 模式位置增益 |
| `gains.gravity_comp.kd` | double[5] | gravity_comp 模式速度增益 |
| `gains.moving.kp` | double[5] | moving 模式位置增益 |
| `gains.moving.kd` | double[5] | moving 模式速度增益 |
| `gains.loaded.kp` | double[5] | loaded 模式位置增益 |
| `gains.loaded.kd` | double[5] | loaded 模式速度增益 |

启动后初始模式：

```text
current_gains_ = gains.gravity_comp
```

如果希望进入 `moving` 或 `loaded`，需要调用 `/set_controller_mode`。

### 5.5 负载模型服务名参数

| 参数 | 类型 | 默认值 | 作用 |
|---|---|---|---|
| `inverse_dynamics.payload_service` | string | `set_payload_state` | 配置 `SetPayloadState` 服务名 |

### 5.6 相机 TF 参数

| 参数 | 类型 | 作用 |
|---|---|---|
| `camera_extrinsics.parent_frame` | string | 末端相机静态 TF parent |
| `camera_extrinsics.child_frame` | string | 末端相机静态 TF child |
| `camera_extrinsics.pos` | double[3] | 平移，单位 m |
| `camera_extrinsics.quat` | double[4] | 四元数 `[qx, qy, qz, qw]` |
| `dog_camera_extrinsics.parent_frame` | string | 狗头相机静态 TF parent |
| `dog_camera_extrinsics.child_frame` | string | 狗头相机静态 TF child |
| `dog_camera_extrinsics.pos` | double[3] | 平移，单位 m |
| `dog_camera_extrinsics.quat` | double[4] | 四元数 `[qx, qy, qz, qw]` |

## 6. 当前容易误解的配置项

以下参数在 `params.yaml` 或 `control_params.yaml` 中存在，但当前 `control_node.cpp` 没有读取或没有完整使用：

| 参数/配置 | 当前情况 |
|---|---|
| `inverse_dynamics.default_mode` | 当前不读取；启动后固定使用 `gravity_comp` 增益 |
| `inverse_dynamics.state_topic` | 当前不读取；订阅固定为 `/arm2/_lowState/joint` |
| `inverse_dynamics.command_topic` | 当前不读取；发布固定为 `/arm2/_lowCmd/command` |
| `inverse_dynamics.ready_topic` | 当前不读取；订阅固定为 `/robot_driver/ready` |
| `inverse_dynamics.control_rate_hz` | 当前不读取；控制周期固定为 10 ms |
| `inverse_dynamics.angle_window_lower/upper` | 当前 `control_node` 不限幅，限位主要依赖目标生成或底层 driver |
| `inverse_dynamics.command_velocity_limits` | 当前不读取 |
| `inverse_dynamics.command_torque_limits` | 当前不读取 |
| `trajectory_planner.*` | 当前 `control_node` 基本不读取 |
| `gains.teach_pendant` / `gains.teach_drag` | 配置中可能存在，但当前代码没有加载为可切换模式 |

## 7. 接口关系总览

```text
driver/sim
  ├─ /robot_driver/ready           std_msgs/Bool
  └─ /arm2/_lowState/joint         robot_msgs/RobotState
             │
             v
      controller_node
        ├─ /move_joint             robot_msgs/action/MoveJoint
        ├─ /set_controller_mode    robot_msgs/srv/SetControllerMode
        ├─ /get_payload_estimate   robot_msgs/srv/GetPayloadEstimate
        ├─ /set_payload_state      robot_msgs/srv/SetPayloadState
        ├─ /arm2/_lowCmd/command   robot_msgs/RobotCommand
        ├─ /debug/friction_torque  std_msgs/Float32MultiArray
        ├─ /tf                     world -> Link_4
        └─ /tf_static              camera extrinsics
```

## 8. 常用调试命令

```bash
# 查看节点
ros2 node list | grep controller_node
ros2 node info /controller_node

# 查看核心话题
ros2 topic echo /arm2/_lowState/joint
ros2 topic echo /arm2/_lowCmd/command
ros2 topic echo /debug/friction_torque

# 切换控制模式
ros2 service call /set_controller_mode robot_msgs/srv/SetControllerMode "{mode: moving}"

# 发送关节目标
ros2 action send_goal /move_joint robot_msgs/action/MoveJoint \
  "{joint_targets: [0.0, 1.57, -1.57, 0.0, 0.0], num_points: 1, max_velocity: 0.6, max_acceleration: 1.0, blend_radius: 0.0}" \
  --feedback

# 查看 TF
ros2 run tf2_ros tf2_echo world Link_4
ros2 run tf2_ros tf2_echo Link_4 camera_link
```
