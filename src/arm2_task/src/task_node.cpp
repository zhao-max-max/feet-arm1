#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "arm2_task/kinematics_engine.hpp"
#include "arm2_task/task/end_effector_client.hpp"
#include "arm2_task/task/motion_client.hpp"
#include "arm2_task/task/nav_pose_tracker.hpp"
#include "arm2_task/task/nav_task_interface.hpp"
#include "arm2_task/task/perception_client.hpp"
#include "arm2_task/task/task_primitives.hpp"
#include "arm2_task/task/task_sequences.hpp"
#include "arm2_task/task/terminal_task_interface.hpp"
#include "arm2_task/task/three_phase_pipeline.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_joint.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/get_payload_estimate.hpp"
#include "robot_msgs/srv/get_pick_pos.hpp"
#include "robot_msgs/srv/get_place_pos.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "robot_msgs/srv/set_payload_state.hpp"
#include "robot_msgs/srv/set_suction.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TaskNode : public rclcpp::Node
{
public:
  using MoveJoint = robot_msgs::action::MoveJoint;

  TaskNode()
      : Node("task_manager_node")
  {
    // ── URDF & Kinematics ──────────────────────────────────────────────────
    const std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
    const std::string rel_urdf = this->declare_parameter("urdf_path", "urdf/arm2.urdf");
    const std::string urdf = share_dir + "/" + rel_urdf;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const double l1 = this->declare_parameter("robot_geometry.l1", 0.0845);
    const double l2 = this->declare_parameter("robot_geometry.l2", 0.350005);
    const double l3 = this->declare_parameter("robot_geometry.l3", 0.243441);
    const double l4 = this->declare_parameter("robot_geometry.l4", 0.046);

    kin_engine_ = std::make_unique<arm2_task::KinematicsEngine>(
        urdf, arm2_task::RobotGeometry(l1, l2, l3, l4));

    // ── Trajectory Parameters ──────────────────────────────────────────────
    load_presets();
    max_v_ = this->declare_parameter("trajectory_planner.max_velocity", 1.0);
    max_a_ = this->declare_parameter("trajectory_planner.max_acceleration", 2.0);
    dist_threshold_ = this->declare_parameter("trajectory_planner.dist_threshold", 0.1);

    // ── Task Parameters ────────────────────────────────────────────────────
    require_payload_service_ = this->declare_parameter("task.require_payload_service", false);
    require_suction_service_ = this->declare_parameter("task.require_suction_service", false);
    payload_service_name_ =
        this->declare_parameter("task.payload_service", std::string("set_payload_state"));
    payload_default_has_load_ =
        this->declare_parameter("task.payload_default.has_load", true);
    payload_default_mass_ =
        this->declare_parameter("task.payload_default.mass", 0.5);
    payload_default_com_ =
        this->declare_parameter("task.payload_default.com", std::vector<double>{0.0, 0.0, 0.2219});
    normalize_payload_default_com();

    // Grasp / place parameters
    grasp_pitch_ = this->declare_parameter("task_step6.grasp_pitch", -1.57);
    tool_pitch_offset_ = this->declare_parameter("task_step6.tool_pitch_offset", 0.0);
    tool_yaw_offset_ = this->declare_parameter("task_step6.tool_yaw_offset", 0.0);
    object_height_ = this->declare_parameter("task_step6.object_height", 0.05);
    pre_grasp_offset_ = this->declare_parameter("task_step6.pre_grasp_offset", 0.10);
    tool_offset_x_ = this->declare_parameter("task_step6.tool_offset_x", 0.0);
    tool_offset_y_ = this->declare_parameter("task_step6.tool_offset_y", 0.0);
    tool_offset_z_ = this->declare_parameter("task_step6.tool_offset_z", 0.0);
    step6_pick_object_name_ = this->declare_parameter(
        "task_step6.pick_object_name", std::string("box"));
    step6_use_mock_target_ = this->declare_parameter("task_step6.use_mock_target", false);
    step6_mock_x_ = this->declare_parameter("task_step6.mock_x", 0.35);
    step6_mock_y_ = this->declare_parameter("task_step6.mock_y", 0.0);
    step6_mock_z_ = this->declare_parameter("task_step6.mock_z", 0.12);

    pre_place_offset_ = this->declare_parameter("task_place.pre_place_offset", 0.12);
    place_retreat_offset_ = this->declare_parameter("task_place.retreat_offset", 0.15);
    tool_tip_length_ = this->declare_parameter("task_step6.tool_tip_length", 0.0);

    // Place-frame parameters (狗头相机放置)
    place_frame_hover_height_ = this->declare_parameter("task_place_frame.hover_height", 0.25);
    place_frame_contact_offset_ = this->declare_parameter("task_place_frame.contact_offset", 0.0);
    place_frame_name_ = this->declare_parameter("task_place_frame.frame_name", std::string("target_frame"));
    place_frame_roll_sign_ = this->declare_parameter("task_place_frame.roll_sign", 1.0);
    place_frame_use_mock_ = this->declare_parameter("task_place_frame.use_mock_target", false);
    place_frame_mock_x_ = this->declare_parameter("task_place_frame.mock_x", 0.35);
    place_frame_mock_y_ = this->declare_parameter("task_place_frame.mock_y", 0.0);
    place_frame_mock_z_ = this->declare_parameter("task_place_frame.mock_z", 0.0);
    place_frame_mock_yaw_ = this->declare_parameter("task_place_frame.mock_yaw", 0.0);

    // Stack parameters (箱子叠放，task_stack)
    stack_hover_height_ = this->declare_parameter("task_stack.hover_height", 0.05);
    stack_contact_offset_ = this->declare_parameter("task_stack.contact_offset", 0.25);
    stack_service_name_ = this->declare_parameter("task_stack.stack_service", std::string("get_stack_pos"));
    stack_roll_sign_ = this->declare_parameter("task_stack.roll_sign", 1.0);
    stack_use_mock_ = this->declare_parameter("task_stack.use_mock_target", false);
    stack_mock_x_ = this->declare_parameter("task_stack.mock_x", 0.35);
    stack_mock_y_ = this->declare_parameter("task_stack.mock_y", 0.0);
    stack_mock_z_ = this->declare_parameter("task_stack.mock_z", 0.1);
    stack_mock_yaw_ = this->declare_parameter("task_stack.mock_yaw", 0.0);

    const auto nav_state_topic =
        this->declare_parameter("task_nav.state_topic", std::string("/navigation/state"));
    const auto nav_task_ids = this->declare_parameter(
        "task_nav.ids",
        std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8});
    const auto nav_task_x = this->declare_parameter(
        "task_nav.x",
        std::vector<double>{2.425, 3.275, 4.125, 4.975, 2.425, 3.275, 4.125, 4.975});
    const auto nav_task_y = this->declare_parameter(
        "task_nav.y",
        std::vector<double>{-6.65, -6.65, -6.65, -6.65, -7.50, -7.50, -7.50, -7.50});
    const auto nav_task_yaw = this->declare_parameter(
        "task_nav.yaw",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Phase-2 alignment parameters
    const double align_threshold = this->declare_parameter("visual_align.align_threshold", 0.005);
    const int align_max_iters = this->declare_parameter("visual_align.max_iters", 5);

    // ── Publishers & Subscribers ───────────────────────────────────────────
    target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/task/target_pose", 10);

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
        "/arm2/_lowState/joint",
        state_qos,
        [this](const robot_msgs::msg::RobotState::SharedPtr msg)
        {
          if (!msg || msg->motor_state.size() < 5)
          {
            return;
          }
          for (int i = 0; i < 5; ++i)
          {
            if (!msg->motor_state[i].valid)
            {
              return;
            }
          }
          std::lock_guard<std::mutex> lock(mtx_);
          if (q_current_.size() != 5)
          {
            q_current_ = Eigen::VectorXd::Zero(5);
            dq_current_ = Eigen::VectorXd::Zero(5);
          }
          for (int i = 0; i < 5; ++i)
          {
            q_current_[i] = msg->motor_state[i].q;
            dq_current_[i] = msg->motor_state[i].dq;
          }
          has_robot_data_ = true;
        });

    auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    driver_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot_driver/ready",
        ready_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          if (msg)
          {
            driver_ready_.store(msg->data);
          }
        });

    // ── Service & Action Clients ───────────────────────────────────────────
    pick_client_ = this->create_client<robot_msgs::srv::GetPickPos>("get_pick_pos");
    place_client_ = this->create_client<robot_msgs::srv::GetPlacePos>("get_place_pos");
    stack_client_ = this->create_client<robot_msgs::srv::GetPlacePos>("get_stack_pos");
    suction_client_ = this->create_client<robot_msgs::srv::SetSuction>("set_suction");
    mode_client_ = this->create_client<robot_msgs::srv::SetControllerMode>("set_controller_mode");
    payload_client_ = this->create_client<robot_msgs::srv::GetPayloadEstimate>("get_payload_estimate");
    payload_state_client_ = this->create_client<robot_msgs::srv::SetPayloadState>(payload_service_name_);
    move_joint_client_ = rclcpp_action::create_client<MoveJoint>(this, "move_joint");

    motion_client_ = std::make_unique<arm2_task::task::MotionClient>(
        this, move_joint_client_, mode_client_, &is_running_);
    motion_client_->set_trajectory_defaults(max_v_, max_a_, dist_threshold_);
    perception_client_ = std::make_unique<arm2_task::task::PerceptionClient>(
        this, tf_buffer_, pick_client_, place_client_, stack_client_);
    end_effector_client_ = std::make_unique<arm2_task::task::EndEffectorClient>(
        this, suction_client_, payload_client_, payload_state_client_);

    arm2_task::task::TaskPrimitives::Config primitive_config;
    primitive_config.require_suction_service = require_suction_service_;
    primitive_config.pick_object_name = step6_pick_object_name_;
    primitive_config.grasp_pitch = grasp_pitch_;
    primitive_config.tool_pitch_offset = tool_pitch_offset_;
    primitive_config.tool_yaw_offset = tool_yaw_offset_;
    primitive_config.object_height = object_height_;
    primitive_config.pre_grasp_offset = pre_grasp_offset_;
    primitive_config.pre_place_offset = pre_place_offset_;
    primitive_config.place_retreat_offset = place_retreat_offset_;
    primitive_config.tool_offset_x = tool_offset_x_;
    primitive_config.tool_offset_y = tool_offset_y_;
    primitive_config.tool_offset_z = tool_offset_z_;
    primitive_config.tool_tip_length = tool_tip_length_;
    primitive_config.place_frame_hover_height = place_frame_hover_height_;
    primitive_config.place_frame_contact_offset = place_frame_contact_offset_;
    primitive_config.stack_hover_height = stack_hover_height_;
    primitive_config.stack_contact_offset = stack_contact_offset_;
    primitive_config.stack_roll_sign = stack_roll_sign_;
    task_primitives_ = std::make_unique<arm2_task::task::TaskPrimitives>(
        this, kin_engine_.get(), motion_client_.get(), perception_client_.get(),
        end_effector_client_.get(), target_pub_, &presets_, &q_current_, &dq_current_,
        &mtx_, &is_running_, primitive_config);

    arm2_task::task::TaskSequences::Config sequence_config;
    sequence_config.pick_object_name = step6_pick_object_name_;
    sequence_config.place_frame_name = place_frame_name_;
    sequence_config.stack_service_name = stack_service_name_;
    sequence_config.use_mock_grasp_target = step6_use_mock_target_;
    sequence_config.grasp_mock_x = step6_mock_x_;
    sequence_config.grasp_mock_y = step6_mock_y_;
    sequence_config.grasp_mock_z = step6_mock_z_;
    sequence_config.use_mock_place_frame = place_frame_use_mock_;
    sequence_config.place_mock_x = place_frame_mock_x_;
    sequence_config.place_mock_y = place_frame_mock_y_;
    sequence_config.place_mock_z = place_frame_mock_z_;
    sequence_config.place_mock_yaw = place_frame_mock_yaw_;
    sequence_config.use_mock_stack = stack_use_mock_;
    sequence_config.stack_mock_x = stack_mock_x_;
    sequence_config.stack_mock_y = stack_mock_y_;
    sequence_config.stack_mock_z = stack_mock_z_;
    sequence_config.stack_mock_yaw = stack_mock_yaw_;
    task_sequences_ = std::make_unique<arm2_task::task::TaskSequences>(
        this, motion_client_.get(), perception_client_.get(), task_primitives_.get(),
        &presets_, sequence_config);

    arm2_task::task::ThreePhasePipeline::Config three_phase_config;
    three_phase_config.pick_object_name = step6_pick_object_name_;
    three_phase_config.align_threshold = align_threshold;
    three_phase_config.align_max_iters = align_max_iters;
    three_phase_pipeline_ = std::make_unique<arm2_task::task::ThreePhasePipeline>(
        this, kin_engine_.get(), motion_client_.get(), perception_client_.get(),
        task_primitives_.get(), target_pub_, &presets_, &q_current_, &mtx_,
        three_phase_config);

    arm2_task::task::NavPoseTracker::Config nav_pose_config;
    nav_pose_config.state_topic = nav_state_topic;
    const auto nav_task_count = std::min(
        {nav_task_ids.size(), nav_task_x.size(), nav_task_y.size(), nav_task_yaw.size()});
    if (nav_task_count != nav_task_ids.size() ||
        nav_task_count != nav_task_x.size() ||
        nav_task_count != nav_task_y.size() ||
        nav_task_count != nav_task_yaw.size())
    {
      RCLCPP_WARN(
          this->get_logger(),
          "task_nav parameter sizes mismatch: ids=%zu x=%zu y=%zu yaw=%zu. Using first %zu entries.",
          nav_task_ids.size(), nav_task_x.size(), nav_task_y.size(), nav_task_yaw.size(),
          nav_task_count);
    }
    for (std::size_t i = 0; i < nav_task_count; ++i)
    {
      arm2_task::task::PlanarPose point;
      point.id = static_cast<int>(nav_task_ids[i]);
      point.x = nav_task_x[i];
      point.y = nav_task_y[i];
      point.yaw = nav_task_yaw[i];
      nav_pose_config.task_points.push_back(point);
    }
    nav_pose_tracker_ = std::make_unique<arm2_task::task::NavPoseTracker>(
        this, std::move(nav_pose_config));

    nav_interface_ = std::make_unique<arm2_task::task::NavTaskInterface>(
        this, task_sequences_.get(), task_primitives_.get(), nav_pose_tracker_.get(),
        &is_running_);

    arm2_task::task::TerminalTaskInterface::Config terminal_config;
    terminal_config.require_payload_service = require_payload_service_;
    terminal_config.payload_default_has_load = payload_default_has_load_;
    terminal_config.payload_default_mass = payload_default_mass_;
    terminal_config.payload_default_com = payload_default_com_;
    terminal_interface_ = std::make_unique<arm2_task::task::TerminalTaskInterface>(
        this, motion_client_.get(), end_effector_client_.get(), task_primitives_.get(),
        task_sequences_.get(), three_phase_pipeline_.get(), &presets_, &is_running_,
        terminal_config);

    RCLCPP_INFO(this->get_logger(), "Task Node Started.");
  }

  ~TaskNode()
  {
    is_running_.store(false);
    if (task_thread_.joinable())
    {
      task_thread_.join();
    }
  }

  void start()
  {
    // 参数 task.manual_mode=true 时启动交互菜单，默认启动 nav 触发模式
    const bool manual_mode = this->declare_parameter("task.manual_mode", false);
    if (manual_mode)
    {
      task_thread_ = std::thread(&TaskNode::run_task_sequence, this);
    }
    else
    {
      task_thread_ = std::thread(&TaskNode::run_remote_control, this);
    }
  }

private:
  // ═══════════════════════════════════════════════════════════════════════════
  // SECTION 1 — Infrastructure (arm 原有，已实机验证，不得修改)
  // ═══════════════════════════════════════════════════════════════════════════

  bool wait_for_system_ready()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for driver readiness...");
    while (rclcpp::ok() && is_running_.load() && !driver_ready_.load())
    {
      rclcpp::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for first robot joint state...");
    while (rclcpp::ok() && is_running_.load() && !has_robot_data_.load())
    {
      rclcpp::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for control services and action server...");
    while (rclcpp::ok() && is_running_.load())
    {
      const bool mode_ready = mode_client_->wait_for_service(500ms);
      const bool payload_ready = !require_payload_service_ || payload_client_->wait_for_service(500ms);
      const bool suction_ready = !require_suction_service_ || suction_client_->wait_for_service(500ms);
      const bool action_ready = move_joint_client_->wait_for_action_server(500ms);

      if (mode_ready && payload_ready && suction_ready && action_ready)
      {
        if (!require_suction_service_ && !suction_client_->wait_for_service(100ms))
        {
          RCLCPP_INFO(this->get_logger(),
                      "Optional suction service [set_suction] not available; suction commands will be skipped.");
        }
        return true;
      }
    }
    return false;
  }

  void load_presets()
  {
    const std::vector<std::string> preset_names = {"reset", "look_out", "load", "carry"};
    for (const auto &name : preset_names)
    {
      const auto angles_deg =
          this->declare_parameter("presets." + name, std::vector<double>(5, 0.0));
      Eigen::VectorXd q_rad(5);
      for (int i = 0; i < 5; ++i)
      {
        q_rad[i] = angles_deg[i] * M_PI / 180.0;
      }
      presets_[name] = q_rad;
      RCLCPP_INFO(this->get_logger(), "Loaded preset '%s'", name.c_str());
    }
  }

  void normalize_payload_default_com()
  {
    if (payload_default_com_.size() == 3U)
    {
      return;
    }
    RCLCPP_WARN(this->get_logger(),
                "task.payload_default.com size=%zu, expected 3. Falling back to [0.0, 0.0, 0.2219].",
                payload_default_com_.size());
    payload_default_com_ = {0.0, 0.0, 0.2219};
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SECTION 4 — Main Task Loop
  // ═══════════════════════════════════════════════════════════════════════════

  /** nav 触发模式入口。具体服务、事件和状态机由 NavTaskInterface 管理。*/
  void run_remote_control()
  {
    if (!wait_for_system_ready())
    {
      return;
    }
    nav_interface_->run();
  }

  void run_task_sequence()
  {
    if (!wait_for_system_ready())
    {
      return;
    }
    terminal_interface_->run();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Member Variables
  // ═══════════════════════════════════════════════════════════════════════════

  // State
  std::atomic<bool> has_robot_data_{false};
  Eigen::VectorXd q_current_;
  Eigen::VectorXd dq_current_;
  std::mutex mtx_;
  std::atomic<bool> driver_ready_{false};

  // Trajectory parameters
  double max_v_{1.0};
  double max_a_{2.0};
  double dist_threshold_{0.05};

  // Task flags
  bool require_payload_service_{false};
  bool require_suction_service_{false};
  std::string payload_service_name_{"set_payload_state"};
  bool payload_default_has_load_{true};
  double payload_default_mass_{0.5};
  std::vector<double> payload_default_com_{0.0, 0.0, 0.2219};

  // Grasp / place parameters
  double grasp_pitch_{-1.57};
  double tool_pitch_offset_{0.0};
  double tool_yaw_offset_{0.0};
  double object_height_{0.05};
  double pre_grasp_offset_{0.10};
  double pre_place_offset_{0.12};
  double place_retreat_offset_{0.15};
  double tool_offset_x_{0.0};
  double tool_offset_y_{0.0};
  double tool_offset_z_{0.0};
  double tool_tip_length_{0.0};

  // Step 6 auto-grasp parameters
  bool step6_use_mock_target_{false};
  double step6_mock_x_{0.35};
  double step6_mock_y_{0.0};
  double step6_mock_z_{0.12};
  std::string step6_pick_object_name_{"box"};

  // Place-frame parameters (狗头相机放置，task_place_frame)
  double place_frame_hover_height_{0.25};
  double place_frame_contact_offset_{0.0};
  std::string place_frame_name_{"target_frame"};
  double place_frame_roll_sign_{1.0};
  bool place_frame_use_mock_{false};
  double place_frame_mock_x_{0.35};
  double place_frame_mock_y_{0.0};
  double place_frame_mock_z_{0.0};
  double place_frame_mock_yaw_{0.0};

  // Stack parameters (箱子叠放，task_stack)
  double stack_hover_height_{0.05};
  double stack_contact_offset_{0.25};
  std::string stack_service_name_{"get_stack_pos"};
  double stack_roll_sign_{1.0};
  bool stack_use_mock_{false};
  double stack_mock_x_{0.35};
  double stack_mock_y_{0.0};
  double stack_mock_z_{0.1};
  double stack_mock_yaw_{0.0};

  // Presets
  std::map<std::string, Eigen::VectorXd> presets_;

  // Kinematics
  std::unique_ptr<arm2_task::KinematicsEngine> kin_engine_;

  // TF (arm 已验证的 shared_ptr 风格)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers / Subscribers / Clients
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_ready_sub_;
  rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr place_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr stack_client_;
  rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client_;
  rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
  rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_client_;
  rclcpp::Client<robot_msgs::srv::SetPayloadState>::SharedPtr payload_state_client_;
  rclcpp_action::Client<MoveJoint>::SharedPtr move_joint_client_;

  std::unique_ptr<arm2_task::task::MotionClient> motion_client_;
  std::unique_ptr<arm2_task::task::PerceptionClient> perception_client_;
  std::unique_ptr<arm2_task::task::EndEffectorClient> end_effector_client_;
  std::unique_ptr<arm2_task::task::TaskPrimitives> task_primitives_;
  std::unique_ptr<arm2_task::task::TaskSequences> task_sequences_;
  std::unique_ptr<arm2_task::task::ThreePhasePipeline> three_phase_pipeline_;
  std::unique_ptr<arm2_task::task::NavPoseTracker> nav_pose_tracker_;
  std::unique_ptr<arm2_task::task::NavTaskInterface> nav_interface_;
  std::unique_ptr<arm2_task::task::TerminalTaskInterface> terminal_interface_;

  // Thread
  std::atomic<bool> is_running_{true};
  std::thread task_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->start();
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
