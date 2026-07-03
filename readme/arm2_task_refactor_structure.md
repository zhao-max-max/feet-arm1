# arm2_task 拆分后代码结构说明

本文档说明 `arm2_task` 中 `task_node` 拆分后的代码结构、各文件职责、调用链路和后续维护边界。

当前拆分目标是：把原本集中在 `src/arm2_task/src/task_node.cpp` 中的运动控制、感知调用、末端执行器控制、任务流程、导航接口和终端调试接口拆到独立模块中，让 `TaskNode` 回到“节点装配壳”的角色。

## 总体结论

当前拆分已经基本完成：

- `task_node.cpp` 不再直接实现导航状态机、终端菜单、3-Phase 流程、抓放叠运动细节。
- 导航接口已经独立到 `NavTaskInterface`。
- 终端调试接口已经独立到 `TerminalTaskInterface`。
- 3-Phase 调试流程已经独立到 `ThreePhasePipeline`。
- 抓取、放置、叠放等可复用业务流程已经独立到 `TaskSequences`。
- 底层动作原语已经独立到 `TaskPrimitives`。
- ROS action/service client 封装已经拆为 `MotionClient`、`PerceptionClient`、`EndEffectorClient`。

需要注意：当前仍然是一个 ROS2 可执行节点 `task_node`，不是多个独立进程。`NavTaskInterface` 和 `TerminalTaskInterface` 是代码层面的独立接口。如果后续需要，也可以在此基础上继续拆成 `nav_task_node` 和 `terminal_task_node` 两个独立 ROS2 node。

## 文件结构

```text
src/arm2_task/
├── CMakeLists.txt
├── include/arm2_task/task/
│   ├── end_effector_client.hpp
│   ├── motion_client.hpp
│   ├── nav_task_interface.hpp
│   ├── perception_client.hpp
│   ├── pose_utils.hpp
│   ├── task_primitives.hpp
│   ├── task_sequences.hpp
│   ├── terminal_task_interface.hpp
│   └── three_phase_pipeline.hpp
└── src/
    ├── task_node.cpp
    └── task/
        ├── end_effector_client.cpp
        ├── motion_client.cpp
        ├── nav_task_interface.cpp
        ├── perception_client.cpp
        ├── pose_utils.cpp
        ├── task_primitives.cpp
        ├── task_sequences.cpp
        ├── terminal_task_interface.cpp
        └── three_phase_pipeline.cpp
```

## 分层结构

```text
TaskNode
  ├── 装配 ROS 参数、订阅、发布、service/action client
  ├── 创建 MotionClient / PerceptionClient / EndEffectorClient
  ├── 创建 TaskPrimitives / TaskSequences / ThreePhasePipeline
  ├── 创建 NavTaskInterface / TerminalTaskInterface
  └── 根据 task.manual_mode 选择 nav 模式或 terminal 模式

接口层
  ├── NavTaskInterface
  └── TerminalTaskInterface

流程层
  ├── TaskSequences
  └── ThreePhasePipeline

原语层
  └── TaskPrimitives

ROS client 封装层
  ├── MotionClient
  ├── PerceptionClient
  └── EndEffectorClient

纯计算工具
  └── pose_utils
```

## task_node.cpp

路径：

```text
src/arm2_task/src/task_node.cpp
```

当前职责：

- 声明和读取任务相关参数。
- 加载 preset：
  - `reset`
  - `look_out`
  - `load`
  - `carry`
- 创建 TF buffer/listener。
- 创建 `/task/target_pose` publisher。
- 订阅机械臂低层状态：
  - `/arm2/_lowState/joint`
- 订阅驱动 ready 状态：
  - `/robot_driver/ready`
- 创建感知、吸盘、payload、控制模式和关节运动 action client。
- 装配各个拆分后的模块。
- 启动线程：
  - `task.manual_mode=true` 时启动 `TerminalTaskInterface`
  - `task.manual_mode=false` 时启动 `NavTaskInterface`

现在 `TaskNode` 不再负责：

- 不再直接实现终端菜单。
- 不再直接实现 nav 状态机。
- 不再直接实现 3-Phase 流程。
- 不再直接实现抓取、放置、叠放运动。
- 不再直接维护 action 结果状态。

### TaskNode 中保留的核心函数

```cpp
bool wait_for_system_ready();
void load_presets();
void normalize_payload_default_com();
void run_remote_control();
void run_task_sequence();
```

含义：

- `wait_for_system_ready()`：等待驱动 ready、第一帧关节状态、控制服务和 action server 可用。
- `load_presets()`：读取 `presets.*` 参数并转换为弧度。
- `normalize_payload_default_com()`：校验 payload 默认质心参数长度。
- `run_remote_control()`：ready 后进入 `NavTaskInterface::run()`。
- `run_task_sequence()`：ready 后进入 `TerminalTaskInterface::run()`。

## MotionClient

文件：

```text
include/arm2_task/task/motion_client.hpp
src/task/motion_client.cpp
```

职责：

- 封装关节运动 action：
  - action 名称：`move_joint`
  - action 类型：`robot_msgs/action/MoveJoint`
- 封装控制模式切换 service：
  - service 名称：`set_controller_mode`
  - service 类型：`robot_msgs/srv/SetControllerMode`
- 维护 action 执行状态、完成状态和错误信息。

主要接口：

```cpp
void set_trajectory_defaults(double max_velocity, double max_acceleration, double blend_radius);
bool send_move_goal(const std::vector<Eigen::VectorXd> & q_waypoints);
bool send_move_goal(const Eigen::VectorXd & q_single);
bool wait_for_action_completion(std::chrono::seconds timeout = std::chrono::seconds(30));
int request_mode_switch(const std::string & mode_name);
```

使用场景：

- `TaskPrimitives` 用它发送 reset、look_out、grasp、place、stack 等轨迹。
- `TaskSequences` 用它切换 moving/loaded 模式和执行 carry preset。
- `ThreePhasePipeline` 用它做 3-Phase 中的 overhead 对齐和 joint_4 旋转。
- `TerminalTaskInterface` 用它执行菜单中的 preset debug 动作。

## PerceptionClient

文件：

```text
include/arm2_task/task/perception_client.hpp
src/task/perception_client.cpp
```

职责：

- 封装抓取目标感知服务。
- 封装放置框目标感知服务。
- 封装叠放目标感知服务。
- 将服务返回的 `PoseStamped` 通过 TF 转换到 `world` 坐标系。

主要接口：

```cpp
bool call_pick_service_sync(const std::string & object_name, geometry_msgs::msg::Pose * out_pose);
bool call_place_service_sync(const std::string & frame_name, geometry_msgs::msg::Pose * out_pose);
bool call_stack_service_sync(const std::string & frame_name, geometry_msgs::msg::Pose * out_pose);
```

对应 ROS 服务：

```text
get_pick_pos
get_place_pos
get_stack_pos
```

其中 `get_stack_pos` 复用 `robot_msgs/srv/GetPlacePos` 类型。

使用场景：

- `TaskSequences::grasp_from_perception()`
- `TaskSequences::place_from_perception()`
- `TaskSequences::stack_mock_or_perception()`
- `ThreePhasePipeline` 的 Phase1/Phase2 感知闭环

## EndEffectorClient

文件：

```text
include/arm2_task/task/end_effector_client.hpp
src/task/end_effector_client.cpp
```

职责：

- 封装吸盘控制。
- 封装 payload 估计。
- 封装 payload 状态设置。

主要接口：

```cpp
int set_suction(bool activate, bool required);
bool request_payload_estimate(double * out_mass);
int request_payload_state(
  bool has_load,
  bool required,
  bool default_has_load,
  double default_mass,
  const std::vector<double> & default_com);
```

对应 ROS 服务：

```text
set_suction
get_payload_estimate
set_payload_state
```

`required` 参数用于控制服务不可用时的行为：

- `required=true`：服务不可用视为失败。
- `required=false`：服务不可用时允许跳过，便于无吸盘或无 payload 服务的调试场景。

## pose_utils

文件：

```text
include/arm2_task/task/pose_utils.hpp
src/task/pose_utils.cpp
```

职责：

- 提供纯计算工具函数，不依赖 ROS node 状态。
- 用于姿态角归一化、视觉朝向解析、叠放 roll 计算。

主要接口：

```cpp
double normalize_angle(double angle);
double object_yaw_roll(const geometry_msgs::msg::Pose & object_world);
EdgeAlignedRoll compute_edge_aligned_roll(const geometry_msgs::msg::Pose & world_pose);
double compute_stack_tool_roll(
  const geometry_msgs::msg::Pose & box_top_world,
  double roll_sign);
```

说明：

- `object_yaw_roll()` 当前默认返回 0 附近的工具 roll，因为 `do_look_out` 和 IK 都以目标方位对齐 joint_0。
- `compute_edge_aligned_roll()` 用于从视觉返回的方框 orientation 中选择一个边缘对齐角。
- `compute_stack_tool_roll()` 用于叠放任务中根据箱子 yaw 和基座 yaw 计算末端 roll。

## TaskPrimitives

文件：

```text
include/arm2_task/task/task_primitives.hpp
src/task/task_primitives.cpp
```

职责：

`TaskPrimitives` 是“动作原语层”。它封装单个可复用机械臂动作或动作组合，但不直接处理 nav 状态机或终端菜单。

依赖：

- `KinematicsEngine`
- `MotionClient`
- `PerceptionClient`
- `EndEffectorClient`
- `/task/target_pose` publisher
- 当前关节状态 `q_current / dq_current`
- preset map

主要配置：

```cpp
struct Config
{
  bool require_suction_service;
  std::string pick_object_name;
  double grasp_pitch;
  double tool_pitch_offset;
  double tool_yaw_offset;
  double object_height;
  double pre_grasp_offset;
  double pre_place_offset;
  double place_retreat_offset;
  double tool_offset_x;
  double tool_offset_y;
  double tool_offset_z;
  double tool_tip_length;
  double place_frame_hover_height;
  double place_frame_contact_offset;
  double stack_hover_height;
  double stack_contact_offset;
  double stack_roll_sign;
};
```

主要接口：

```cpp
void wait_joints_still(double dq_threshold = 0.02, int timeout_ms = 1000);

void do_reset();
void do_reset_suction();
void do_load();
void do_look_out(const geometry_msgs::msg::Pose & target);
void do_suction_on();
void do_suction_off();

bool do_grasp_move(const geometry_msgs::msg::Pose & target, double tool_roll);
bool do_grasp_move(const geometry_msgs::msg::Pose & target);
bool do_place_move(const geometry_msgs::msg::Pose & target);
bool do_place_move_with_orientation(const geometry_msgs::msg::Pose & frame_world);
bool do_stack_move_with_orientation(const geometry_msgs::msg::Pose & box_top_world);

bool do_full_grasp(const geometry_msgs::msg::Pose & target);
bool do_full_grasp_aligned(const geometry_msgs::msg::Pose & target);
bool do_full_place(const geometry_msgs::msg::Pose & target);
```

典型动作含义：

- `do_reset()`：吸盘关闭，切 moving，移动到 reset preset。
- `do_reset_suction()`：保持吸盘状态，移动到 reset 后切 idle。
- `do_load()`：移动到 load preset。
- `do_look_out()`：joint_0 朝向目标 XY，其余关节使用 look_out preset。
- `do_suction_on()`：等待稳定，吸盘 ON，切 loaded。
- `do_suction_off()`：吸盘 OFF，切 moving。
- `do_grasp_move()`：计算 pre-grasp 和 grasp 两个 IK 点并执行轨迹。
- `do_place_move()`：计算 pre-place 和 place 两个 IK 点，完成后垂直后退。
- `do_place_move_with_orientation()`：用于狗头相机返回方框 orientation 的放置。
- `do_stack_move_with_orientation()`：用于叠放任务。
- `do_full_grasp_aligned()`：多次采样感知结果，取中位数，按方框边缘方向抓取。

## TaskSequences

文件：

```text
include/arm2_task/task/task_sequences.hpp
src/task/task_sequences.cpp
```

职责：

`TaskSequences` 是“业务流程层”。它把多个原语组合成可复用任务流程，供 nav 和 terminal 共同调用。

它不直接读取 stdin，也不直接处理 nav event。

主要配置：

```cpp
struct Config
{
  std::string pick_object_name;
  std::string place_frame_name;
  std::string stack_service_name;

  bool use_mock_grasp_target;
  double grasp_mock_x;
  double grasp_mock_y;
  double grasp_mock_z;

  bool use_mock_place_frame;
  double place_mock_x;
  double place_mock_y;
  double place_mock_z;
  double place_mock_yaw;

  bool use_mock_stack;
  double stack_mock_x;
  double stack_mock_y;
  double stack_mock_z;
  double stack_mock_yaw;
};
```

主要接口：

```cpp
bool grasp_from_perception();
bool grasp_mock_or_perception();
bool grasp_pose(const geometry_msgs::msg::Pose & target, bool aligned);

bool place_from_perception();
bool place_mock_or_perception();
bool place_pose(const geometry_msgs::msg::Pose & frame_pose);

bool stack_mock_or_perception();
bool stack_pose(const geometry_msgs::msg::Pose & box_top_pose);

bool move_to_carry_loaded();
```

典型流程：

### 自动抓取

```text
moving
  -> look_out forward
  -> wait joints still
  -> call get_pick_pos
  -> do_full_grasp_aligned
```

### 自动放置

```text
call get_place_pos 或使用 mock pose
  -> moving
  -> do_place_move_with_orientation
  -> moving
```

### 自动叠放

```text
call get_stack_pos 或使用 mock pose
  -> moving
  -> do_stack_move_with_orientation
  -> moving
```

### Carry 归位

```text
moving
  -> carry preset
  -> loaded
```

## ThreePhasePipeline

文件：

```text
include/arm2_task/task/three_phase_pipeline.hpp
src/task/three_phase_pipeline.cpp
```

职责：

封装 3-Phase 终端调试流程。

它仍然允许终端输入确认，因此它属于“调试流程模块”，不是 nav 自动流程。

主要配置：

```cpp
struct Config
{
  std::string pick_object_name;
  double align_threshold;
  int align_max_iters;
};
```

主要接口：

```cpp
bool do_grasp();
bool do_place();
```

内部阶段：

```cpp
bool phase1_get_coarse_target(geometry_msgs::msg::Pose & target_world);
bool phase2_align(geometry_msgs::msg::Pose & target_world);
bool phase3_grasp_descend(const geometry_msgs::msg::Pose & target_world);
bool phase3_place_descend(const geometry_msgs::msg::Pose & target_world);
```

### 3-Phase 抓取流程

```text
Phase1:
  move to look_out
  choose real sensor / manual input / abort
  get coarse target

Phase2:
  move to load overhead pose
  repeatedly call get_pick_pos
  adjust joint_0 to reduce XY error
  stop when error < visual_align.align_threshold

Phase3:
  rotate joint_4 to -90 deg
  wait terminal confirmation
  do_grasp_move
  suction ON
```

### 3-Phase 放置流程

```text
Phase1:
  get coarse target

Phase2:
  overhead XY alignment

Phase3:
  rotate joint_4 to -90 deg
  wait terminal confirmation
  do_place_move
  suction OFF
```

## NavTaskInterface

文件：

```text
include/arm2_task/task/nav_task_interface.hpp
src/task/nav_task_interface.cpp
```

职责：

封装导航自动任务入口。

它拥有：

- `/arm/mission_event` service server
- `/navigation/arm_event` service client
- nav 任务状态机
- busy 防重入保护

ROS 接口：

```text
Service Server:
  /arm/mission_event
  std_srvs/srv/Trigger

Service Client:
  /navigation/arm_event
  navigation/srv/StringCommand
```

内部状态：

```cpp
arm2_task::TaskState state_{arm2_task::TaskState::IDLE};
std::atomic<bool> remote_busy_{false};
bool pending_trigger_{false};
```

主要接口：

```cpp
void run();
```

主要内部函数：

```cpp
void send_nav_event(const std::string & event);
bool do_grasp_sequence();
bool do_place_sequence();
```

当前 nav 状态机：

```text
IDLE
  receive /arm/mission_event
  -> do_grasp_sequence()
  -> send "grabbed"
  -> move_to_carry_loaded()
  -> send "completed"
  -> HOLDING

HOLDING
  receive /arm/mission_event
  -> currently warns: place sequence not yet wired to nav
```

说明：

- 抓取链路已经接入 nav。
- 放置链路的函数 `do_place_sequence()` 已存在，但状态机中暂未启用。
- 后续如果 nav 支持放置任务点，可在 `HOLDING` 分支启用 `do_place_sequence()`，成功后切回 `IDLE`。

## TerminalTaskInterface

文件：

```text
include/arm2_task/task/terminal_task_interface.hpp
src/task/terminal_task_interface.cpp
```

职责：

封装终端菜单调试入口。

它拥有：

- 控制台菜单打印
- stdin 非阻塞轮询
- 手动输入目标 pose
- payload estimate / payload state 调试命令
- 调用 `TaskSequences`、`TaskPrimitives`、`ThreePhasePipeline`

主要接口：

```cpp
void run();
```

菜单命令：

```text
1:  Reset
2:  Joint preset A
3:  Joint preset B
4:  Auto place
5:  Manual place
6:  Auto grasp
7:  Manual grasp
8:  Release
9:  Carry reset
10: Move to load preset
11: Estimate payload
12: 3-Phase Grasp
13: 3-Phase Place
14: Auto Stack
15: Manual Stack
16: Enable payload model
17: Clear payload model
0:  Exit
```

说明：

- 原来 `task_node.cpp` 中的大段终端菜单逻辑已经迁移到这里。
- 终端调试链路现在与 nav 入口分离。
- 终端模式仍然复用与 nav 相同的 `TaskSequences` 和 `TaskPrimitives`，因此底层动作行为一致。

## 运行入口链路

### manual_mode=false：导航模式

```text
TaskNode::start()
  -> run_remote_control()
    -> wait_for_system_ready()
    -> NavTaskInterface::run()
      -> wait /arm/mission_event
      -> TaskSequences::grasp_from_perception()
      -> TaskSequences::move_to_carry_loaded()
      -> send /navigation/arm_event
```

### manual_mode=true：终端模式

```text
TaskNode::start()
  -> run_task_sequence()
    -> wait_for_system_ready()
    -> TerminalTaskInterface::run()
      -> print menu
      -> wait stdin command
      -> call TaskPrimitives / TaskSequences / ThreePhasePipeline
```

## 参数流向

`TaskNode` 仍然负责参数声明和读取，然后把参数填入各组件的 `Config`。

### TaskPrimitives::Config

来源主要是：

```text
task.require_suction_service
task_step6.*
task_place.*
task_place_frame.*
task_stack.*
```

用途：

- 控制吸盘服务是否 required。
- 控制抓取、放置、叠放的几何偏移。
- 控制 hover/contact/retreat 高度。
- 控制 stack roll sign。

### TaskSequences::Config

来源主要是：

```text
task_step6.pick_object_name
task_step6.use_mock_target
task_step6.mock_*
task_place_frame.frame_name
task_place_frame.use_mock_target
task_place_frame.mock_*
task_stack.stack_service
task_stack.use_mock_target
task_stack.mock_*
```

用途：

- 决定自动流程调用真实感知还是 mock target。
- 指定 pick/place/stack 服务请求目标名。

### ThreePhasePipeline::Config

来源：

```text
task_step6.pick_object_name
visual_align.align_threshold
visual_align.max_iters
```

用途：

- 3-Phase 感知目标名。
- Phase2 XY 闭环收敛阈值和迭代次数。

### TerminalTaskInterface::Config

来源：

```text
task.require_payload_service
task.payload_default.has_load
task.payload_default.mass
task.payload_default.com
```

用途：

- 终端菜单中的 payload estimate / enable / clear 调试命令。

## CMake 构建关系

`task_node` 可执行文件现在包含：

```cmake
add_executable(task_node
  src/task_node.cpp
  src/task/pose_utils.cpp
  src/task/motion_client.cpp
  src/task/nav_task_interface.cpp
  src/task/perception_client.cpp
  src/task/end_effector_client.cpp
  src/task/task_primitives.cpp
  src/task/task_sequences.cpp
  src/task/terminal_task_interface.cpp
  src/task/three_phase_pipeline.cpp
)
```

依赖仍然通过 `ament_target_dependencies(task_node ...)` 管理。

## 当前拆分状态评价

已经完成的拆分：

- 底层 ROS client 封装完成。
- 运动原语层拆分完成。
- 业务流程层拆分完成。
- 3-Phase 调试流程拆分完成。
- nav 自动接口拆分完成。
- terminal 调试接口拆分完成。
- `TaskNode` 已从巨型业务节点降级为装配壳。

还可以继续优化但不是当前必须：

- `TaskNode` 构造函数仍然包含较多参数声明，可继续抽成 `TaskConfig`。
- 组件创建逻辑可以继续抽成 `TaskContext` 或 `TaskComponentFactory`。
- `TaskPrimitives` 仍然较大，可继续拆成：
  - `BasicMotionPrimitives`
  - `GraspPrimitives`
  - `PlacePrimitives`
  - `StackPrimitives`
- 如果需要更彻底隔离 nav 和 terminal，可把当前接口类提升为两个独立 ROS2 可执行节点：
  - `nav_task_node`
  - `terminal_task_node`

## 维护建议

新增功能时建议按以下规则放置代码：

- 新增 ROS service/action 调用：优先放到 `MotionClient`、`PerceptionClient` 或 `EndEffectorClient`。
- 新增单个机械臂动作：放到 `TaskPrimitives`。
- 新增抓取/放置/叠放组合流程：放到 `TaskSequences`。
- 新增 3-Phase 调试步骤：放到 `ThreePhasePipeline`。
- 新增 nav 自动任务状态：放到 `NavTaskInterface`。
- 新增终端菜单命令：放到 `TerminalTaskInterface`。
- 不建议把新业务逻辑重新写回 `task_node.cpp`。

`task_node.cpp` 后续应只负责：

- 参数读取
- ROS 接口创建
- 组件装配
- 系统 ready 检查
- 选择 nav 或 terminal 入口

## 验证命令

拆分后建议用以下命令验证：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select arm2_task
git diff --check
```

当前拆分后的代码已经通过 `colcon build --packages-select arm2_task` 编译验证。
