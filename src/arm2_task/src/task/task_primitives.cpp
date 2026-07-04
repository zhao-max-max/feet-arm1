#include "arm2_task/task/task_primitives.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "arm2_task/task/pose_utils.hpp"
#include "arm2_task/task/timing_events.hpp"

using namespace std::chrono_literals;

namespace arm2_task::task
{

TaskPrimitives::TaskPrimitives(
  rclcpp::Node * node,
  arm2_task::KinematicsEngine * kin_engine,
  MotionClient * motion_client,
  PerceptionClient * perception_client,
  EndEffectorClient * end_effector_client,
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub,
  const std::map<std::string, Eigen::VectorXd> * presets,
  Eigen::VectorXd * q_current,
  Eigen::VectorXd * dq_current,
  std::mutex * state_mutex,
  const std::atomic<bool> * is_running,
  Config config)
: node_(node),
  kin_engine_(kin_engine),
  motion_client_(motion_client),
  perception_client_(perception_client),
  end_effector_client_(end_effector_client),
  target_pub_(std::move(target_pub)),
  presets_(presets),
  q_current_(q_current),
  dq_current_(dq_current),
  state_mutex_(state_mutex),
  is_running_(is_running),
  config_(std::move(config))
{
  timing_event_pub_ = create_timing_event_publisher(node_);
}

bool TaskPrimitives::task_is_running() const
{
  return is_running_ == nullptr || is_running_->load();
}

bool TaskPrimitives::send_move_goal(const std::vector<Eigen::VectorXd> & q_waypoints)
{
  return motion_client_->send_move_goal(q_waypoints);
}

bool TaskPrimitives::send_move_goal(const Eigen::VectorXd & q_single)
{
  return motion_client_->send_move_goal(q_single);
}

bool TaskPrimitives::wait_for_action_completion(std::chrono::seconds timeout)
{
  return motion_client_->wait_for_action_completion(timeout);
}

int TaskPrimitives::request_mode_switch(const std::string & mode_name)
{
  return motion_client_->request_mode_switch(mode_name);
}

int TaskPrimitives::set_suction(bool activate)
{
  return end_effector_client_->set_suction(activate, config_.require_suction_service);
}

double TaskPrimitives::get_object_yaw(const geometry_msgs::msg::Pose & object_world) const
{
  return object_yaw_roll(object_world);
}

double TaskPrimitives::get_box_edge_roll(const geometry_msgs::msg::Pose & world_pose)
{
  const auto result = compute_edge_aligned_roll(world_pose);
  if (result.identity_orientation) {
    RCLCPP_INFO(node_->get_logger(), "[box_edge_roll] identity orientation, roll=0");
    return 0.0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[box_edge_roll] alpha=%.3f chosen=%.3f edge_yaw=%.3f base_yaw=%.3f roll=%.3f rad (%.1f deg)",
    result.alpha, result.chosen, result.edge_yaw, result.base_yaw,
    result.roll, result.roll * 180.0 / M_PI);
  return result.roll;
}

double TaskPrimitives::get_frame_yaw(const geometry_msgs::msg::Pose & frame_world) const
{
  const auto result = compute_edge_aligned_roll(frame_world);
  if (result.identity_orientation) {
    RCLCPP_INFO(node_->get_logger(), "[get_frame_yaw] identity orientation, roll=0");
    return 0.0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[get_frame_yaw] alpha=%.3f chosen=%.3f edge_yaw=%.3f base_yaw=%.3f roll=%.3f rad (%.1f deg)",
    result.alpha, result.chosen, result.edge_yaw, result.base_yaw,
    result.roll, result.roll * 180.0 / M_PI);
  return result.roll;
}

double TaskPrimitives::apply_roll_continuity(double roll)
{
  last_grasp_roll_ = roll;
  return roll;
}

Eigen::VectorXd TaskPrimitives::current_q_snapshot() const
{
  std::lock_guard<std::mutex> lock(*state_mutex_);
  return *q_current_;
}

Eigen::VectorXd TaskPrimitives::current_dq_snapshot() const
{
  std::lock_guard<std::mutex> lock(*state_mutex_);
  return *dq_current_;
}

void TaskPrimitives::wait_joints_still(double dq_threshold, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  auto stable_since = std::chrono::steady_clock::time_point{};
  bool stable_seen = false;

  while (rclcpp::ok() && task_is_running()) {
    const Eigen::VectorXd dq_snapshot = current_dq_snapshot();
    const bool has_dq = (dq_snapshot.size() == 5);
    const bool all_still =
      has_dq && (dq_snapshot.array().abs() < dq_threshold).all();
    const auto now = std::chrono::steady_clock::now();

    if (all_still) {
      if (!stable_seen) {
        stable_since = now;
        stable_seen = true;
      } else if (now - stable_since >= 150ms) {
        return;
      }
    } else {
      stable_seen = false;
    }

    if (now >= deadline) {
      double max_abs_dq = -1.0;
      if (has_dq) {
        max_abs_dq = dq_snapshot.cwiseAbs().maxCoeff();
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "[wait_joints_still] timeout after %d ms, continue with max|dq|=%.4f",
        timeout_ms, max_abs_dq);
      return;
    }

    rclcpp::sleep_for(20ms);
  }
}

void TaskPrimitives::do_reset()
{
  publish_timing_event(node_, timing_event_pub_, "action", "reset", "begin");
  RCLCPP_INFO(node_->get_logger(), "[reset] suction OFF -> moving -> reset -> moving");
  set_suction(false);
  if (!request_mode_switch("moving")) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "reset", "end", "ok=0,reason=mode_moving_failed");
    return;
  }
  if (!presets_->count("reset")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'reset' not found!");
    publish_timing_event(
      node_, timing_event_pub_, "action", "reset", "end", "ok=0,reason=missing_preset");
    return;
  }
  bool ok = true;
  if (send_move_goal({presets_->at("reset")})) {
    ok = wait_for_action_completion();
  } else {
    ok = false;
  }
  request_mode_switch("moving");
  publish_timing_event(
    node_, timing_event_pub_, "action", "reset", "end", std::string("ok=") + timing_bool(ok));
}

void TaskPrimitives::do_reset_suction()
{
  publish_timing_event(node_, timing_event_pub_, "action", "reset_suction", "begin");
  RCLCPP_INFO(node_->get_logger(), "[reset_suction] moving -> reset -> idle");
  if (!request_mode_switch("moving")) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "reset_suction", "end",
      "ok=0,reason=mode_moving_failed");
    return;
  }
  if (!presets_->count("reset")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'reset' not found!");
    publish_timing_event(
      node_, timing_event_pub_, "action", "reset_suction", "end",
      "ok=0,reason=missing_preset");
    return;
  }
  bool ok = true;
  if (send_move_goal({presets_->at("reset")})) {
    ok = wait_for_action_completion();
  } else {
    ok = false;
  }
  request_mode_switch("idle");
  publish_timing_event(
    node_, timing_event_pub_, "action", "reset_suction", "end",
    std::string("ok=") + timing_bool(ok));
}

void TaskPrimitives::do_load()
{
  publish_timing_event(node_, timing_event_pub_, "action", "load", "begin");
  if (!presets_->count("load")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'load' not found!");
    publish_timing_event(
      node_, timing_event_pub_, "action", "load", "end", "ok=0,reason=missing_preset");
    return;
  }
  bool ok = true;
  if (send_move_goal({presets_->at("load")})) {
    ok = wait_for_action_completion();
  } else {
    ok = false;
  }
  publish_timing_event(
    node_, timing_event_pub_, "action", "load", "end", std::string("ok=") + timing_bool(ok));
}

bool TaskPrimitives::do_look_out(const geometry_msgs::msg::Pose & target)
{
  std::ostringstream detail;
  detail << "target_x=" << target.position.x << ",target_y=" << target.position.y;
  publish_timing_event(node_, timing_event_pub_, "action", "look_out", "begin", detail.str());

  if (!presets_->count("look_out")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'look_out' not found!");
    publish_timing_event(
      node_, timing_event_pub_, "action", "look_out", "end",
      detail.str() + ",ok=0,reason=missing_preset");
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(), "[look_out] yaw toward (%.3f, %.3f)",
    target.position.x, target.position.y);

  auto goal_q = presets_->at("look_out");
  goal_q[0] = std::atan2(target.position.y, target.position.x);
  goal_q[4] = 0.0;
  detail << ",yaw=" << goal_q[0];
  bool ok = false;
  if (send_move_goal({goal_q})) {
    ok = wait_for_action_completion();
  }
  publish_timing_event(
    node_, timing_event_pub_, "action", "look_out", "end",
    detail.str() + ",ok=" + timing_bool(ok));
  return ok;
}

void TaskPrimitives::do_suction_on()
{
  publish_timing_event(node_, timing_event_pub_, "action", "suction_on", "begin");
  RCLCPP_INFO(node_->get_logger(), "[suction] waiting 400ms for arm to settle...");
  rclcpp::sleep_for(400ms);
  const bool suction_ok = set_suction(true) != 0;
  rclcpp::sleep_for(500ms);
  const bool mode_ok = request_mode_switch("loaded") != 0;
  publish_timing_event(
    node_, timing_event_pub_, "action", "suction_on", "end",
    std::string("ok=") + timing_bool(suction_ok && mode_ok) +
    ",suction_ok=" + timing_bool(suction_ok) +
    ",mode_ok=" + timing_bool(mode_ok));
}

void TaskPrimitives::do_suction_off()
{
  publish_timing_event(node_, timing_event_pub_, "action", "suction_off", "begin");
  RCLCPP_INFO(node_->get_logger(), "[suction] OFF -> mode=moving");
  const bool suction_ok = set_suction(false) != 0;
  const bool mode_ok = request_mode_switch("moving") != 0;
  publish_timing_event(
    node_, timing_event_pub_, "action", "suction_off", "end",
    std::string("ok=") + timing_bool(suction_ok && mode_ok) +
    ",suction_ok=" + timing_bool(suction_ok) +
    ",mode_ok=" + timing_bool(mode_ok));
}

bool TaskPrimitives::do_grasp_move(
  const geometry_msgs::msg::Pose & target,
  double tool_roll)
{
  std::ostringstream detail;
  detail << "target_x=" << target.position.x
         << ",target_y=" << target.position.y
         << ",target_z=" << target.position.z
         << ",tool_roll=" << tool_roll;
  publish_timing_event(node_, timing_event_pub_, "action", "grasp_move", "begin", detail.str());

  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const double q0 = std::atan2(target.position.y, target.position.x);
  const double cos_q0 = std::cos(q0);
  const double sin_q0 = std::sin(q0);
  const Eigen::Vector3d suction_target(
    target.position.x + config_.tool_offset_x * cos_q0 - config_.tool_offset_y * sin_q0,
    target.position.y + config_.tool_offset_x * sin_q0 + config_.tool_offset_y * cos_q0,
    target.position.z + config_.object_height + config_.tool_offset_z);

  const Eigen::Vector3d tip_dir(
    std::cos(q0) * std::cos(pitch),
    std::sin(q0) * std::cos(pitch),
    std::sin(pitch));
  const Eigen::Vector3d ee_target = suction_target - config_.tool_tip_length * tip_dir;
  const Eigen::Vector3d pre_target(
    ee_target.x(), ee_target.y(), ee_target.z() + config_.pre_grasp_offset);

  Eigen::VectorXd q_pre(5), q_grasp(5);
  if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[grasp_move] IK failed for pre-grasp (%.3f, %.3f, %.3f)",
      pre_target.x(), pre_target.y(), pre_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "grasp_move", "end",
      detail.str() + ",ok=0,reason=pre_grasp_ik_failed");
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_grasp)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[grasp_move] IK failed for grasp (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "grasp_move", "end",
      detail.str() + ",ok=0,reason=grasp_ik_failed");
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_grasp[0] += config_.tool_yaw_offset;

  RCLCPP_INFO(
    node_->get_logger(),
    "[grasp_move] pre-z=%.3f grasp-z=%.3f pitch=%.2f roll=%.2f",
    pre_target.z(), ee_target.z(), pitch, tool_roll);

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_grasp})) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "grasp_move", "end",
      detail.str() + ",ok=0,reason=send_goal_failed");
    return false;
  }
  const bool ok = wait_for_action_completion();
  publish_timing_event(
    node_, timing_event_pub_, "action", "grasp_move", "end",
    detail.str() + ",ok=" + timing_bool(ok));
  return ok;
}

bool TaskPrimitives::do_grasp_move(const geometry_msgs::msg::Pose & target)
{
  return do_grasp_move(target, get_object_yaw(target));
}

bool TaskPrimitives::do_place_move(const geometry_msgs::msg::Pose & target)
{
  std::ostringstream detail;
  detail << "target_x=" << target.position.x
         << ",target_y=" << target.position.y
         << ",target_z=" << target.position.z;
  publish_timing_event(node_, timing_event_pub_, "action", "place_move", "begin", detail.str());

  const double tool_roll = get_object_yaw(target);
  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const double q0 = std::atan2(target.position.y, target.position.x);
  const double cos_q0 = std::cos(q0);
  const double sin_q0 = std::sin(q0);
  const Eigen::Vector3d suction_target(
    target.position.x + config_.tool_offset_x * cos_q0 - config_.tool_offset_y * sin_q0,
    target.position.y + config_.tool_offset_x * sin_q0 + config_.tool_offset_y * cos_q0,
    target.position.z + config_.object_height + config_.tool_offset_z);

  const Eigen::Vector3d tip_dir(
    std::cos(q0) * std::cos(pitch),
    std::sin(q0) * std::cos(pitch),
    std::sin(pitch));
  const Eigen::Vector3d ee_target = suction_target - config_.tool_tip_length * tip_dir;
  const Eigen::Vector3d pre_target(
    ee_target.x(), ee_target.y(), ee_target.z() + config_.pre_place_offset);

  Eigen::VectorXd q_pre(5), q_place(5);
  if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_move] IK failed for pre-place (%.3f, %.3f, %.3f)",
      pre_target.x(), pre_target.y(), pre_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_move", "end",
      detail.str() + ",ok=0,reason=pre_place_ik_failed");
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_place)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_move] IK failed for place (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_move", "end",
      detail.str() + ",ok=0,reason=place_ik_failed");
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_place[0] += config_.tool_yaw_offset;

  RCLCPP_INFO(
    node_->get_logger(),
    "[place_move] pre-z=%.3f place-z=%.3f pitch=%.2f roll=%.2f",
    pre_target.z(), ee_target.z(), pitch, tool_roll);

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_place})) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_move", "end",
      detail.str() + ",ok=0,reason=send_goal_failed");
    return false;
  }
  if (!wait_for_action_completion()) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_move", "end",
      detail.str() + ",ok=0,reason=move_failed");
    return false;
  }

  const Eigen::VectorXd q_current_snap = current_q_snapshot();
  if (q_current_snap.size() == 5) {
    const auto fk = kin_engine_->forwardKinematics(q_current_snap);
    Eigen::Vector3d retreat_pos = fk.translation();
    retreat_pos.z() += config_.place_retreat_offset;

    Eigen::VectorXd q_retreat(5);
    if (kin_engine_->solveIK(retreat_pos, pitch, tool_roll, q_retreat)) {
      q_retreat[0] += config_.tool_yaw_offset;
      RCLCPP_INFO(
        node_->get_logger(), "[place_move] retreating %.2fm upward",
        config_.place_retreat_offset);
      if (send_move_goal({q_retreat})) {
        wait_for_action_completion();
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "[place_move] retreat IK failed, skipping.");
    }
  }
  publish_timing_event(
    node_, timing_event_pub_, "action", "place_move", "end",
    detail.str() + ",ok=1");
  return true;
}

bool TaskPrimitives::do_place_move_with_orientation(
  const geometry_msgs::msg::Pose & frame_world)
{
  std::ostringstream detail;
  detail << "frame_x=" << frame_world.position.x
         << ",frame_y=" << frame_world.position.y
         << ",frame_z=" << frame_world.position.z;
  publish_timing_event(node_, timing_event_pub_, "action", "place_frame", "begin", detail.str());

  const double tool_roll = get_frame_yaw(frame_world);
  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const Eigen::Vector3d ee_target(
    frame_world.position.x,
    frame_world.position.y,
    frame_world.position.z + config_.place_frame_contact_offset);
  const Eigen::Vector3d pre_target(
    ee_target.x(), ee_target.y(), ee_target.z() + config_.place_frame_hover_height);

  RCLCPP_INFO(
    node_->get_logger(), "[place_frame] ee_z=%.3f pre_z=%.3f pitch=%.2f roll=%.2f",
    ee_target.z(), pre_target.z(), pitch, tool_roll);

  Eigen::VectorXd q_pre(5), q_place(5);
  if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_frame] IK failed for pre-place (%.3f, %.3f, %.3f)",
      pre_target.x(), pre_target.y(), pre_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_frame", "end",
      detail.str() + ",ok=0,reason=pre_place_ik_failed");
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_place)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_frame] IK failed for place (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_frame", "end",
      detail.str() + ",ok=0,reason=place_ik_failed");
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_place[0] += config_.tool_yaw_offset;

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_place})) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_frame", "end",
      detail.str() + ",ok=0,reason=send_goal_failed");
    return false;
  }
  if (!wait_for_action_completion()) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_frame", "end",
      detail.str() + ",ok=0,reason=move_failed");
    return false;
  }

  const Eigen::VectorXd q_snap = current_q_snapshot();
  RCLCPP_INFO(node_->get_logger(), "[place_frame] suction OFF before retreat");
  rclcpp::sleep_for(200ms);
  do_suction_off();
  rclcpp::sleep_for(300ms);

  if (q_snap.size() == 5) {
    const auto fk = kin_engine_->forwardKinematics(q_snap);
    Eigen::Vector3d retreat_pos = fk.translation();
    retreat_pos.z() += config_.place_retreat_offset;

    Eigen::VectorXd q_retreat(5);
    if (kin_engine_->solveIK(retreat_pos, pitch, tool_roll, q_retreat)) {
      q_retreat[0] += config_.tool_yaw_offset;
      RCLCPP_INFO(
        node_->get_logger(), "[place_frame] retreating %.2fm upward",
        config_.place_retreat_offset);
      if (send_move_goal(std::vector<Eigen::VectorXd>{q_retreat})) {
        wait_for_action_completion();
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "[place_frame] retreat IK failed, skipping.");
    }
  }
  publish_timing_event(
    node_, timing_event_pub_, "action", "place_frame", "end",
    detail.str() + ",ok=1");
  return true;
}

bool TaskPrimitives::do_stack_move_with_orientation(
  const geometry_msgs::msg::Pose & box_top_world)
{
  std::ostringstream detail;
  detail << "box_top_x=" << box_top_world.position.x
         << ",box_top_y=" << box_top_world.position.y
         << ",box_top_z=" << box_top_world.position.z;
  publish_timing_event(node_, timing_event_pub_, "action", "stack_move", "begin", detail.str());

  const double tool_roll = compute_stack_tool_roll(box_top_world, config_.stack_roll_sign);
  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const Eigen::Vector3d ee_target(
    box_top_world.position.x,
    box_top_world.position.y,
    box_top_world.position.z + config_.stack_contact_offset);
  const Eigen::Vector3d pre_target(
    ee_target.x(), ee_target.y(), ee_target.z() + config_.stack_hover_height);

  RCLCPP_INFO(
    node_->get_logger(), "[stack] ee_z=%.3f pre_z=%.3f pitch=%.2f roll=%.2f",
    ee_target.z(), pre_target.z(), pitch, tool_roll);

  Eigen::VectorXd q_pre(5), q_stack(5);
  if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[stack] IK failed for pre-stack (%.3f, %.3f, %.3f)",
      pre_target.x(), pre_target.y(), pre_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "stack_move", "end",
      detail.str() + ",ok=0,reason=pre_stack_ik_failed");
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_stack)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[stack] IK failed for stack (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "stack_move", "end",
      detail.str() + ",ok=0,reason=stack_ik_failed");
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_stack[0] += config_.tool_yaw_offset;

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_stack})) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "stack_move", "end",
      detail.str() + ",ok=0,reason=send_goal_failed");
    return false;
  }
  if (!wait_for_action_completion()) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "stack_move", "end",
      detail.str() + ",ok=0,reason=move_failed");
    return false;
  }

  const Eigen::VectorXd q_snap = current_q_snapshot();
  RCLCPP_INFO(node_->get_logger(), "[stack] suction OFF before retreat");
  rclcpp::sleep_for(200ms);
  do_suction_off();
  rclcpp::sleep_for(300ms);

  if (q_snap.size() == 5) {
    const auto fk = kin_engine_->forwardKinematics(q_snap);
    Eigen::Vector3d retreat_pos = fk.translation();
    retreat_pos.z() += config_.place_retreat_offset;

    Eigen::VectorXd q_retreat(5);
    if (kin_engine_->solveIK(retreat_pos, pitch, tool_roll, q_retreat)) {
      q_retreat[0] += config_.tool_yaw_offset;
      RCLCPP_INFO(
        node_->get_logger(), "[stack] retreating %.2fm upward",
        config_.place_retreat_offset);
      if (send_move_goal(std::vector<Eigen::VectorXd>{q_retreat})) {
        wait_for_action_completion();
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "[stack] retreat IK failed, skipping.");
    }
  }
  publish_timing_event(
    node_, timing_event_pub_, "action", "stack_move", "end",
    detail.str() + ",ok=1");
  return true;
}

bool TaskPrimitives::do_full_grasp(const geometry_msgs::msg::Pose & target)
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "full_grasp", "begin");
  target_pub_->publish(target);
  request_mode_switch("moving");
  do_look_out(target);
  wait_joints_still(0.02, 200);

  if (!do_grasp_move(target)) {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "full_grasp", "end",
      "ok=0,reason=grasp_move_failed");
    return false;
  }
  do_suction_on();
  publish_timing_event(node_, timing_event_pub_, "sequence", "full_grasp", "end", "ok=1");
  return true;
}

bool TaskPrimitives::do_full_grasp_aligned(const geometry_msgs::msg::Pose & target)
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "full_grasp_aligned", "begin");
  target_pub_->publish(target);
  request_mode_switch("moving");
  do_look_out(target);
  wait_joints_still(0.02, 200);

  constexpr int kSamples = 3;
  std::vector<geometry_msgs::msg::Pose> samples;
  publish_timing_event(node_, timing_event_pub_, "perception", "pick_samples_aligned", "begin");
  for (int i = 0; i < kSamples; ++i) {
    geometry_msgs::msg::Pose p;
    if (perception_client_->call_pick_service_sync(config_.pick_object_name, &p)) {
      samples.push_back(p);
    } else {
      RCLCPP_WARN(node_->get_logger(), "[grasp_aligned] sample %d/%d failed", i + 1, kSamples);
    }
  }
  publish_timing_event(
    node_, timing_event_pub_, "perception", "pick_samples_aligned", "end",
    "success_count=" + std::to_string(samples.size()) + ",sample_count=" + std::to_string(kSamples));

  geometry_msgs::msg::Pose refined = target;
  double roll = get_box_edge_roll(target);

  if (!samples.empty()) {
    std::vector<double> xs, ys, zs, rolls;
    for (const auto & s : samples) {
      xs.push_back(s.position.x);
      ys.push_back(s.position.y);
      zs.push_back(s.position.z);
      rolls.push_back(get_box_edge_roll(s));
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());
    std::sort(rolls.begin(), rolls.end());
    const int m = static_cast<int>(xs.size() / 2);
    refined = samples[m];
    refined.position.x = xs[m];
    refined.position.y = ys[m];
    refined.position.z = zs[m];
    roll = rolls[rolls.size() / 2];
    roll = apply_roll_continuity(roll);

    RCLCPP_INFO(
      node_->get_logger(),
      "[grasp_aligned] %zu samples, median pos=(%.3f,%.3f,%.3f) roll=%.3f rad (%.1f deg)",
      samples.size(), refined.position.x, refined.position.y, refined.position.z,
      roll, roll * 180.0 / M_PI);
  } else {
    RCLCPP_WARN(node_->get_logger(), "[grasp_aligned] all perception failed, using 1st result");
  }

  if (!do_grasp_move(refined, roll)) {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "full_grasp_aligned", "end",
      "ok=0,reason=grasp_move_failed");
    return false;
  }
  do_suction_on();
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "full_grasp_aligned", "end", "ok=1");
  return true;
}

bool TaskPrimitives::do_grasp_from_current_view()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "grasp_from_current_view", "begin");
  request_mode_switch("moving");
  wait_joints_still(0.02, 200);

  constexpr int kSamples = 3;
  std::vector<geometry_msgs::msg::Pose> samples;
  publish_timing_event(node_, timing_event_pub_, "perception", "pick_samples_current_view", "begin");
  for (int i = 0; i < kSamples; ++i) {
    geometry_msgs::msg::Pose p;
    if (perception_client_->call_pick_service_sync(config_.pick_object_name, &p)) {
      samples.push_back(p);
    } else {
      RCLCPP_WARN(node_->get_logger(), "[grasp_current_view] sample %d/%d failed", i + 1, kSamples);
    }
  }
  publish_timing_event(
    node_, timing_event_pub_, "perception", "pick_samples_current_view", "end",
    "success_count=" + std::to_string(samples.size()) + ",sample_count=" + std::to_string(kSamples));

  if (samples.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[grasp_current_view] all perception samples failed.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "grasp_from_current_view", "end",
      "ok=0,reason=all_perception_failed");
    return false;
  }

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<double> rolls;
  xs.reserve(samples.size());
  ys.reserve(samples.size());
  zs.reserve(samples.size());
  rolls.reserve(samples.size());
  for (const auto & sample : samples) {
    xs.push_back(sample.position.x);
    ys.push_back(sample.position.y);
    zs.push_back(sample.position.z);
    rolls.push_back(get_box_edge_roll(sample));
  }

  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  std::sort(zs.begin(), zs.end());
  std::sort(rolls.begin(), rolls.end());
  const auto median = samples.size() / 2;

  geometry_msgs::msg::Pose refined = samples[median];
  refined.position.x = xs[median];
  refined.position.y = ys[median];
  refined.position.z = zs[median];
  const double roll = apply_roll_continuity(rolls[median]);

  target_pub_->publish(refined);
  RCLCPP_INFO(
    node_->get_logger(),
    "[grasp_current_view] %zu samples from ready yaw, median pos=(%.3f,%.3f,%.3f) roll=%.3f rad (%.1f deg)",
    samples.size(), refined.position.x, refined.position.y, refined.position.z,
    roll, roll * 180.0 / M_PI);

  if (!do_grasp_move(refined, roll)) {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "grasp_from_current_view", "end",
      "ok=0,reason=grasp_move_failed");
    return false;
  }
  do_suction_on();
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "grasp_from_current_view", "end", "ok=1");
  return true;
}

bool TaskPrimitives::do_full_place(const geometry_msgs::msg::Pose & target)
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "full_place", "begin");
  target_pub_->publish(target);
  request_mode_switch("moving");
  do_look_out(target);

  RCLCPP_INFO(node_->get_logger(), "[place] moving to pre-place then place...");
  if (!do_place_move(target)) {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "full_place", "end",
      "ok=0,reason=place_move_failed");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[place] suction OFF");
  rclcpp::sleep_for(200ms);
  do_suction_off();
  rclcpp::sleep_for(300ms);

  RCLCPP_INFO(node_->get_logger(), "[place] done.");
  publish_timing_event(node_, timing_event_pub_, "sequence", "full_place", "end", "ok=1");
  return true;
}

}  // namespace arm2_task::task
