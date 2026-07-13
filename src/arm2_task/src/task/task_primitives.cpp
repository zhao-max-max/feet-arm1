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

geometry_msgs::msg::Pose TaskPrimitives::clamp_to_min_reach(
  const geometry_msgs::msg::Pose & pose) const
{
  const double min_reach = config_.dog_half_length + config_.box_half_length;
  if (min_reach <= 0.0) {
    return pose;
  }
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double dist = std::sqrt(x * x + y * y);
  if (dist >= min_reach) {
    return pose;
  }
  geometry_msgs::msg::Pose out = pose;
  if (dist < 1e-6) {
    out.position.x = min_reach;
    out.position.y = 0.0;
  } else {
    const double scale = min_reach / dist;
    out.position.x = x * scale;
    out.position.y = y * scale;
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "[place] Target (%.3f, %.3f) too close (dist=%.3f < min_reach=%.3f); clamped to (%.3f, %.3f).",
    x, y, dist, min_reach, out.position.x, out.position.y);
  return out;
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
      } else if (now - stable_since >= 100ms) {
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

void TaskPrimitives::do_store_pose()
{
  publish_timing_event(node_, timing_event_pub_, "action", "store_pose", "begin");
  RCLCPP_INFO(
    node_->get_logger(),
    "[store_pose] moving -> hover above store -> descend to store preset -> moving");

  if (!request_mode_switch("moving")) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "store_pose", "end", "ok=0,reason=mode_moving_failed");
    return;
  }
  if (!presets_->count("store")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'store' not found!");
    publish_timing_event(
      node_, timing_event_pub_, "action", "store_pose", "end", "ok=0,reason=missing_preset");
    return;
  }

  const Eigen::VectorXd & q_store = presets_->at("store");

  // Use FK to find the end-effector position at the store preset,
  // then build a hover waypoint directly above it.
  const auto store_se3 = kin_engine_->forwardKinematics(q_store);
  const Eigen::Vector3d store_pos = store_se3.translation();
  const Eigen::Vector3d hover_pos(
    store_pos.x(), store_pos.y(), store_pos.z() + config_.store_hover_offset);

  // Keep the same orientation (pitch/roll) as the store preset for the hover IK.
  // Extract pitch from the store joint angles: pitch ≈ sum of joint angles that
  // form the wrist pitch. Use the same grasp_pitch convention for consistency.
  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const double roll  = q_store[4];  // joint_4 of the store preset

  Eigen::VectorXd q_hover(5);
  if (!kin_engine_->solveIK(hover_pos, pitch, roll, q_hover)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[store_pose] IK failed for hover (%.3f, %.3f, %.3f); going directly to store preset.",
      hover_pos.x(), hover_pos.y(), hover_pos.z());
    // Fall back: go straight to store preset without hover
    bool ok = send_move_goal(std::vector<Eigen::VectorXd>{q_store}) && wait_for_action_completion();
    request_mode_switch("moving");
    publish_timing_event(
      node_, timing_event_pub_, "action", "store_pose", "end",
      std::string("ok=") + timing_bool(ok) + ",hover=0");
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[store_pose] hover-z=%.3f store-z=%.3f pitch=%.2f roll=%.2f roll_offset=%.2f",
    hover_pos.z(), store_pos.z(), pitch, roll, config_.store_roll_offset);

  // Apply roll offset to both waypoints (relative to store preset joint_4)
  Eigen::VectorXd q_hover_final = q_hover;
  Eigen::VectorXd q_store_final = q_store;
  q_hover_final[4] += config_.store_roll_offset;
  q_store_final[4] += config_.store_roll_offset;

  bool ok = send_move_goal(std::vector<Eigen::VectorXd>{q_hover_final, q_store_final}) && wait_for_action_completion();
  request_mode_switch("moving");
  publish_timing_event(
    node_, timing_event_pub_, "action", "store_pose", "end",
    std::string("ok=") + timing_bool(ok) + ",hover=1");
}

void TaskPrimitives::do_dog_suction_on()
{
  publish_timing_event(node_, timing_event_pub_, "action", "dog_suction_on", "begin");
  RCLCPP_INFO(node_->get_logger(), "[dog_suction] ON");
  rclcpp::sleep_for(std::chrono::milliseconds(400));
  const bool ok = end_effector_client_->set_dog_suction(true, false) != 0;
  publish_timing_event(
    node_, timing_event_pub_, "action", "dog_suction_on", "end",
    std::string("ok=") + timing_bool(ok));
}

void TaskPrimitives::do_dog_suction_off()
{
  publish_timing_event(node_, timing_event_pub_, "action", "dog_suction_off", "begin");
  RCLCPP_INFO(node_->get_logger(), "[dog_suction] OFF");
  const bool ok = end_effector_client_->set_dog_suction(false, false) != 0;
  publish_timing_event(
    node_, timing_event_pub_, "action", "dog_suction_off", "end",
    std::string("ok=") + timing_bool(ok));
}

bool TaskPrimitives::do_look_out(const geometry_msgs::msg::Pose & target)
{
  std::ostringstream detail;
  detail << "target_x=" << target.position.x << ",target_y=" << target.position.y;
  publish_timing_event(node_, timing_event_pub_, "action", "look_out", "begin", detail.str());

  request_mode_switch("moving");

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

bool TaskPrimitives::do_pre_place_pivot(const geometry_msgs::msg::Pose & target)
{
  std::ostringstream detail;
  detail << "target_x=" << target.position.x << ",target_y=" << target.position.y;
  publish_timing_event(node_, timing_event_pub_, "action", "pre_place_pivot", "begin", detail.str());

  if (!presets_->count("pre_place")) {
    RCLCPP_WARN(node_->get_logger(), "[pre_place_pivot] Preset 'pre_place' not found, skipping.");
    publish_timing_event(
      node_, timing_event_pub_, "action", "pre_place_pivot", "end",
      detail.str() + ",ok=0,reason=missing_preset");
    return false;
  }

  // 降速保证平滑，完成后恢复默认速度
  motion_client_->set_trajectory_defaults(
    config_.pre_place_velocity, config_.default_acceleration, config_.default_blend_radius);

  const auto pre_place_q = presets_->at("pre_place");
  if (!send_move_goal(std::vector<Eigen::VectorXd>{pre_place_q}) || !wait_for_action_completion()) {
    RCLCPP_WARN(node_->get_logger(), "[pre_place_pivot] Move to pre_place failed, continuing anyway.");
    motion_client_->set_trajectory_defaults(
      config_.default_velocity, config_.default_acceleration, config_.default_blend_radius);
    publish_timing_event(
      node_, timing_event_pub_, "action", "pre_place_pivot", "end",
      detail.str() + ",ok=0,reason=pre_place_move_failed");
    return false;
  }

  const double target_yaw = std::atan2(target.position.y, target.position.x);

  // place_ready 是途径点，和 pivot 合并成一个 move_goal，由 blending 平滑过渡
  if (presets_->count("place_ready")) {
    auto place_ready_q = presets_->at("place_ready");
    auto pivot_q = place_ready_q;
    pivot_q[0] = target_yaw;
    detail << ",yaw=" << target_yaw;
    RCLCPP_INFO(
      node_->get_logger(),
      "[pre_place_pivot] place_ready + pivot joint0=%.3f rad toward (%.3f, %.3f)",
      target_yaw, target.position.x, target.position.y);
    if (!send_move_goal(std::vector<Eigen::VectorXd>{place_ready_q, pivot_q}) || !wait_for_action_completion()) {
      RCLCPP_WARN(node_->get_logger(), "[pre_place_pivot] place_ready+pivot failed, continuing anyway.");
      motion_client_->set_trajectory_defaults(
        config_.default_velocity, config_.default_acceleration, config_.default_blend_radius);
      publish_timing_event(
        node_, timing_event_pub_, "action", "pre_place_pivot", "end",
        detail.str() + ",ok=0,reason=pivot_move_failed");
      return false;
    }
  } else {
    auto pivot_q = pre_place_q;
    pivot_q[0] = target_yaw;
    detail << ",yaw=" << target_yaw;
    RCLCPP_INFO(
      node_->get_logger(),
      "[pre_place_pivot] pivot joint0=%.3f rad toward (%.3f, %.3f)",
      target_yaw, target.position.x, target.position.y);
    if (!send_move_goal(std::vector<Eigen::VectorXd>{pivot_q}) || !wait_for_action_completion()) {
      RCLCPP_WARN(node_->get_logger(), "[pre_place_pivot] Pivot failed, continuing anyway.");
      motion_client_->set_trajectory_defaults(
        config_.default_velocity, config_.default_acceleration, config_.default_blend_radius);
      publish_timing_event(
        node_, timing_event_pub_, "action", "pre_place_pivot", "end",
        detail.str() + ",ok=0,reason=pivot_move_failed");
      return false;
    }
  }

  motion_client_->set_trajectory_defaults(
    config_.default_velocity, config_.default_acceleration, config_.default_blend_radius);
  publish_timing_event(
    node_, timing_event_pub_, "action", "pre_place_pivot", "end",
    detail.str() + ",ok=1");
  return true;
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
  q_pre[4] += config_.grasp_roll_offset;
  q_grasp[4] += config_.grasp_roll_offset;

  RCLCPP_INFO(
    node_->get_logger(),
    "[grasp_move] pre-z=%.3f grasp-z=%.3f pitch=%.2f roll=%.2f roll_offset=%.2f",
    pre_target.z(), ee_target.z(), pitch, tool_roll, config_.grasp_roll_offset);

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
  const geometry_msgs::msg::Pose frame = clamp_to_min_reach(frame_world);
  std::ostringstream detail;
  detail << "frame_x=" << frame.position.x
         << ",frame_y=" << frame.position.y
         << ",frame_z=" << frame.position.z;
  publish_timing_event(node_, timing_event_pub_, "action", "place_frame", "begin", detail.str());

  do_pre_place_pivot(frame);

  const double tool_roll = get_frame_yaw(frame);
  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const Eigen::Vector3d ee_target(
    frame.position.x,
    frame.position.y,
    frame.position.z + config_.place_frame_contact_offset);
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

bool TaskPrimitives::do_place_move_with_direct_height(
  const geometry_msgs::msg::Pose & target)
{
  const geometry_msgs::msg::Pose tgt = clamp_to_min_reach(target);
  std::ostringstream detail;
  detail << "target_x=" << tgt.position.x
         << ",target_y=" << tgt.position.y
         << ",target_z=" << tgt.position.z;
  publish_timing_event(
    node_, timing_event_pub_, "action", "place_direct_height", "begin", detail.str());

  do_pre_place_pivot(tgt);

  const double tool_roll = get_frame_yaw(tgt);
  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const Eigen::Vector3d ee_target(
    tgt.position.x,
    tgt.position.y,
    tgt.position.z);
  const Eigen::Vector3d pre_target(
    ee_target.x(), ee_target.y(), ee_target.z() + config_.place_frame_hover_height);

  RCLCPP_INFO(
    node_->get_logger(),
    "[place_direct] ee=(%.3f, %.3f, %.3f) pre_z=%.3f pitch=%.2f roll=%.2f",
    ee_target.x(), ee_target.y(), ee_target.z(), pre_target.z(), pitch, tool_roll);

  Eigen::VectorXd q_pre(5), q_place(5);
  if (!kin_engine_->solveIK(pre_target, pitch, tool_roll, q_pre)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_direct] IK failed for pre-place (%.3f, %.3f, %.3f)",
      pre_target.x(), pre_target.y(), pre_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_direct_height", "end",
      detail.str() + ",ok=0,reason=pre_place_ik_failed");
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_place)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_direct] IK failed for place (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_direct_height", "end",
      detail.str() + ",ok=0,reason=place_ik_failed");
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_place[0] += config_.tool_yaw_offset;

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_place})) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_direct_height", "end",
      detail.str() + ",ok=0,reason=send_goal_failed");
    return false;
  }
  if (!wait_for_action_completion()) {
    publish_timing_event(
      node_, timing_event_pub_, "action", "place_direct_height", "end",
      detail.str() + ",ok=0,reason=move_failed");
    return false;
  }

  const Eigen::VectorXd q_snap = current_q_snapshot();
  RCLCPP_INFO(node_->get_logger(), "[place_direct] suction OFF before retreat");
  rclcpp::sleep_for(200ms);
  do_suction_off();
  rclcpp::sleep_for(300ms);

  std::vector<Eigen::VectorXd> post_release_waypoints;
  if (q_snap.size() == 5) {
    const auto fk = kin_engine_->forwardKinematics(q_snap);
    Eigen::Vector3d retreat_pos = fk.translation();
    retreat_pos.z() += config_.place_retreat_offset;

    Eigen::VectorXd q_retreat(5);
    if (kin_engine_->solveIK(retreat_pos, pitch, tool_roll, q_retreat)) {
      q_retreat[0] += config_.tool_yaw_offset;
      RCLCPP_INFO(
        node_->get_logger(), "[place_direct] adding retreat waypoint %.2fm upward",
        config_.place_retreat_offset);
      post_release_waypoints.push_back(q_retreat);
    } else {
      RCLCPP_WARN(node_->get_logger(), "[place_direct] retreat IK failed, falling back to reset only.");
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "[place_direct] no joint snapshot for retreat, falling back to reset only.");
  }

  if (presets_ != nullptr && presets_->count("reset")) {
    post_release_waypoints.push_back(presets_->at("reset"));
  } else {
    RCLCPP_WARN(node_->get_logger(), "[place_direct] Preset 'reset' not found; post-place reset skipped.");
  }

  if (!post_release_waypoints.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[place_direct] post-release move_joint with %zu waypoint(s).",
      post_release_waypoints.size());
    if (!send_move_goal(post_release_waypoints)) {
      publish_timing_event(
        node_, timing_event_pub_, "action", "place_direct_height", "end",
        detail.str() + ",ok=0,reason=post_release_send_failed");
      return false;
    }
    if (!wait_for_action_completion()) {
      publish_timing_event(
        node_, timing_event_pub_, "action", "place_direct_height", "end",
        detail.str() + ",ok=0,reason=post_release_move_failed");
      return false;
    }
  }

  publish_timing_event(
    node_, timing_event_pub_, "action", "place_direct_height", "end",
    detail.str() + ",ok=1");
  return true;
}

bool TaskPrimitives::do_stack_move_with_orientation(
  const geometry_msgs::msg::Pose & box_top_world)
{
  const geometry_msgs::msg::Pose box_top = clamp_to_min_reach(box_top_world);
  std::ostringstream detail;
  detail << "box_top_x=" << box_top.position.x
         << ",box_top_y=" << box_top.position.y
         << ",box_top_z=" << box_top.position.z;
  publish_timing_event(node_, timing_event_pub_, "action", "stack_move", "begin", detail.str());

  do_pre_place_pivot(box_top);

  const double tool_roll = compute_stack_tool_roll(box_top, config_.stack_roll_sign);
  const double pitch = config_.grasp_pitch + config_.tool_pitch_offset;
  const Eigen::Vector3d ee_target(
    box_top.position.x,
    box_top.position.y,
    box_top.position.z + config_.stack_contact_offset);
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
  const auto median = samples.size() / 2;

  geometry_msgs::msg::Pose refined = samples[median];
  refined.position.x = xs[median];
  refined.position.y = ys[median];
  refined.position.z = zs[median];

  // Re-align joint_0 to the visual target (same as terminal do_full_grasp_aligned),
  // then re-sample for accurate roll from a head-on view.
  do_look_out(refined);
  wait_joints_still(0.02, 200);

  std::vector<geometry_msgs::msg::Pose> aligned_samples;
  publish_timing_event(
    node_, timing_event_pub_, "perception", "pick_samples_aligned_view", "begin");
  for (int i = 0; i < kSamples; ++i) {
    geometry_msgs::msg::Pose p;
    if (perception_client_->call_pick_service_sync(config_.pick_object_name, &p)) {
      aligned_samples.push_back(p);
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "[grasp_current_view] aligned sample %d/%d failed", i + 1, kSamples);
    }
  }
  publish_timing_event(
    node_, timing_event_pub_, "perception", "pick_samples_aligned_view", "end",
    "success_count=" + std::to_string(aligned_samples.size()) +
    ",sample_count=" + std::to_string(kSamples));

  // Use aligned samples if available, otherwise fall back to original samples
  if (!aligned_samples.empty()) {
    std::vector<double> axs, ays, azs, arolls;
    for (const auto & s : aligned_samples) {
      axs.push_back(s.position.x);
      ays.push_back(s.position.y);
      azs.push_back(s.position.z);
      // Use refined.position for base_yaw so it matches joint_0 set by do_look_out(refined).
      // The perception sample position may differ slightly from refined, which would shift
      // base_yaw away from actual joint_0 and produce a wrong roll.
      geometry_msgs::msg::Pose s_roll = s;
      s_roll.position.x = refined.position.x;
      s_roll.position.y = refined.position.y;
      arolls.push_back(get_box_edge_roll(s_roll));
    }
    std::sort(axs.begin(), axs.end());
    std::sort(ays.begin(), ays.end());
    std::sort(azs.begin(), azs.end());
    std::sort(arolls.begin(), arolls.end());
    const auto am = aligned_samples.size() / 2;
    refined = aligned_samples[am];
    refined.position.x = axs[am];
    refined.position.y = ays[am];
    refined.position.z = azs[am];
    rolls.clear();
    rolls = arolls;
  }

  std::sort(rolls.begin(), rolls.end());
  const double roll = apply_roll_continuity(rolls[rolls.size() / 2]);

  target_pub_->publish(refined);
  RCLCPP_INFO(
    node_->get_logger(),
    "[grasp_current_view] %zu aligned samples, median pos=(%.3f,%.3f,%.3f) roll=%.3f rad (%.1f deg)",
    aligned_samples.empty() ? samples.size() : aligned_samples.size(),
    refined.position.x, refined.position.y, refined.position.z,
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
