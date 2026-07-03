#include "arm2_task/task/task_primitives.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "arm2_task/task/pose_utils.hpp"

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
  RCLCPP_INFO(node_->get_logger(), "[reset] suction OFF -> moving -> reset -> moving");
  set_suction(false);
  if (!request_mode_switch("moving")) {
    return;
  }
  if (!presets_->count("reset")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'reset' not found!");
    return;
  }
  if (send_move_goal({presets_->at("reset")})) {
    wait_for_action_completion();
  }
  request_mode_switch("moving");
}

void TaskPrimitives::do_reset_suction()
{
  RCLCPP_INFO(node_->get_logger(), "[reset_suction] moving -> reset -> idle");
  if (!request_mode_switch("moving")) {
    return;
  }
  if (!presets_->count("reset")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'reset' not found!");
    return;
  }
  if (send_move_goal({presets_->at("reset")})) {
    wait_for_action_completion();
  }
  request_mode_switch("idle");
}

void TaskPrimitives::do_load()
{
  if (!presets_->count("load")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'load' not found!");
    return;
  }
  if (send_move_goal({presets_->at("load")})) {
    wait_for_action_completion();
  }
}

bool TaskPrimitives::do_look_out(const geometry_msgs::msg::Pose & target)
{
  if (!presets_->count("look_out")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'look_out' not found!");
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(), "[look_out] yaw toward (%.3f, %.3f)",
    target.position.x, target.position.y);

  auto goal_q = presets_->at("look_out");
  goal_q[0] = std::atan2(target.position.y, target.position.x);
  goal_q[4] = 0.0;
  if (send_move_goal({goal_q})) {
    return wait_for_action_completion();
  }
  return false;
}

void TaskPrimitives::do_suction_on()
{
  RCLCPP_INFO(node_->get_logger(), "[suction] waiting 400ms for arm to settle...");
  rclcpp::sleep_for(400ms);
  set_suction(true);
  rclcpp::sleep_for(500ms);
  request_mode_switch("loaded");
}

void TaskPrimitives::do_suction_off()
{
  RCLCPP_INFO(node_->get_logger(), "[suction] OFF -> mode=moving");
  set_suction(false);
  request_mode_switch("moving");
}

bool TaskPrimitives::do_grasp_move(
  const geometry_msgs::msg::Pose & target,
  double tool_roll)
{
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
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_grasp)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[grasp_move] IK failed for grasp (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_grasp[0] += config_.tool_yaw_offset;

  RCLCPP_INFO(
    node_->get_logger(),
    "[grasp_move] pre-z=%.3f grasp-z=%.3f pitch=%.2f roll=%.2f",
    pre_target.z(), ee_target.z(), pitch, tool_roll);

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_grasp})) {
    return false;
  }
  return wait_for_action_completion();
}

bool TaskPrimitives::do_grasp_move(const geometry_msgs::msg::Pose & target)
{
  return do_grasp_move(target, get_object_yaw(target));
}

bool TaskPrimitives::do_place_move(const geometry_msgs::msg::Pose & target)
{
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
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_place)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_move] IK failed for place (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_place[0] += config_.tool_yaw_offset;

  RCLCPP_INFO(
    node_->get_logger(),
    "[place_move] pre-z=%.3f place-z=%.3f pitch=%.2f roll=%.2f",
    pre_target.z(), ee_target.z(), pitch, tool_roll);

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_place})) {
    return false;
  }
  if (!wait_for_action_completion()) {
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
  return true;
}

bool TaskPrimitives::do_place_move_with_orientation(
  const geometry_msgs::msg::Pose & frame_world)
{
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
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_place)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[place_frame] IK failed for place (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_place[0] += config_.tool_yaw_offset;

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_place})) {
    return false;
  }
  if (!wait_for_action_completion()) {
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
  return true;
}

bool TaskPrimitives::do_stack_move_with_orientation(
  const geometry_msgs::msg::Pose & box_top_world)
{
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
    return false;
  }
  if (!kin_engine_->solveIK(ee_target, pitch, tool_roll, q_stack)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[stack] IK failed for stack (%.3f, %.3f, %.3f)",
      ee_target.x(), ee_target.y(), ee_target.z());
    return false;
  }
  q_pre[0] += config_.tool_yaw_offset;
  q_stack[0] += config_.tool_yaw_offset;

  if (!send_move_goal(std::vector<Eigen::VectorXd>{q_pre, q_stack})) {
    return false;
  }
  if (!wait_for_action_completion()) {
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
  return true;
}

bool TaskPrimitives::do_full_grasp(const geometry_msgs::msg::Pose & target)
{
  target_pub_->publish(target);
  request_mode_switch("moving");
  do_look_out(target);
  wait_joints_still(0.02, 200);

  if (!do_grasp_move(target)) {
    return false;
  }
  do_suction_on();
  return true;
}

bool TaskPrimitives::do_full_grasp_aligned(const geometry_msgs::msg::Pose & target)
{
  target_pub_->publish(target);
  request_mode_switch("moving");
  do_look_out(target);
  wait_joints_still(0.02, 200);

  constexpr int kSamples = 3;
  std::vector<geometry_msgs::msg::Pose> samples;
  for (int i = 0; i < kSamples; ++i) {
    geometry_msgs::msg::Pose p;
    if (perception_client_->call_pick_service_sync(config_.pick_object_name, &p)) {
      samples.push_back(p);
    } else {
      RCLCPP_WARN(node_->get_logger(), "[grasp_aligned] sample %d/%d failed", i + 1, kSamples);
    }
  }

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
    return false;
  }
  do_suction_on();
  return true;
}

bool TaskPrimitives::do_full_place(const geometry_msgs::msg::Pose & target)
{
  target_pub_->publish(target);
  request_mode_switch("moving");
  do_look_out(target);

  RCLCPP_INFO(node_->get_logger(), "[place] moving to pre-place then place...");
  if (!do_place_move(target)) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[place] suction OFF");
  rclcpp::sleep_for(200ms);
  do_suction_off();
  rclcpp::sleep_for(300ms);

  RCLCPP_INFO(node_->get_logger(), "[place] done.");
  return true;
}

}  // namespace arm2_task::task
