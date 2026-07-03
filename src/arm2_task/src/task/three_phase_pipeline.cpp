#include "arm2_task/task/three_phase_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

using namespace std::chrono_literals;

namespace arm2_task::task
{

ThreePhasePipeline::ThreePhasePipeline(
  rclcpp::Node * node,
  arm2_task::KinematicsEngine * kin_engine,
  MotionClient * motion_client,
  PerceptionClient * perception_client,
  TaskPrimitives * primitives,
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub,
  const std::map<std::string, Eigen::VectorXd> * presets,
  Eigen::VectorXd * q_current,
  std::mutex * state_mutex,
  Config config)
: node_(node),
  kin_engine_(kin_engine),
  motion_client_(motion_client),
  perception_client_(perception_client),
  primitives_(primitives),
  target_pub_(std::move(target_pub)),
  presets_(presets),
  q_current_(q_current),
  state_mutex_(state_mutex),
  config_(std::move(config))
{
}

geometry_msgs::msg::Pose ThreePhasePipeline::make_forward_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

Eigen::VectorXd ThreePhasePipeline::current_q_snapshot() const
{
  std::lock_guard<std::mutex> lock(*state_mutex_);
  return *q_current_;
}

bool ThreePhasePipeline::phase1_get_coarse_target(geometry_msgs::msg::Pose & target_world)
{
  RCLCPP_INFO(node_->get_logger(), "[Phase1] Scan: moving to look_out pose...");
  motion_client_->request_mode_switch("moving");

  primitives_->do_look_out(make_forward_pose());
  primitives_->wait_joints_still(0.02, 200);

  RCLCPP_INFO(node_->get_logger(), "[Phase1] At look_out. Select input mode:");
  std::cout << "\n[Phase1/Scan] Sensor input: (r)eal sensor / (m)anual input / (a)bort: ";
  char c = 'a';
  std::cin >> c;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  if (c == 'a') {
    RCLCPP_WARN(node_->get_logger(), "[Phase1] Aborted.");
    return false;
  }

  if (c == 'r') {
    return perception_client_->call_pick_service_sync(config_.pick_object_name, &target_world);
  }

  double mx = 0.0;
  double my = 0.0;
  double mz = 0.0;
  std::cout << "  Enter target position in world frame  x y z (m): ";
  if (!(std::cin >> mx >> my >> mz)) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    RCLCPP_WARN(node_->get_logger(), "[Phase1] Invalid input.");
    return false;
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  target_world.position.x = mx;
  target_world.position.y = my;
  target_world.position.z = mz;
  target_world.orientation.w = 1.0;
  RCLCPP_INFO(
    node_->get_logger(), "[Phase1] Manual target: world=(%.3f, %.3f, %.3f)",
    mx, my, mz);
  return true;
}

bool ThreePhasePipeline::phase2_align(geometry_msgs::msg::Pose & target_world)
{
  RCLCPP_INFO(node_->get_logger(), "[Phase2] Look-down: moving to overhead (load) pose...");
  motion_client_->request_mode_switch("moving");

  if (!presets_->count("load")) {
    RCLCPP_ERROR(node_->get_logger(), "Preset 'load' not found!");
    return false;
  }

  auto q_overhead = presets_->at("load");
  q_overhead[0] = std::atan2(target_world.position.y, target_world.position.x);
  q_overhead[4] = 0.0;
  if (!motion_client_->send_move_goal({q_overhead})) {
    return false;
  }
  if (!motion_client_->wait_for_action_completion()) {
    return false;
  }
  primitives_->wait_joints_still(0.02, 200);

  const Eigen::VectorXd q0 = current_q_snapshot();
  if (q0.size() != 5) {
    RCLCPP_ERROR(node_->get_logger(), "[Phase2] No valid joint state.");
    return false;
  }
  const double fixed_z = kin_engine_->forwardKinematics(q0).translation().z();
  RCLCPP_INFO(
    node_->get_logger(),
    "[Phase2] Overhead Z fixed at %.3f m. Starting alignment loop...",
    fixed_z);

  for (int iter = 0; iter < config_.align_max_iters; ++iter) {
    geometry_msgs::msg::Pose detected;
    if (!perception_client_->call_pick_service_sync(config_.pick_object_name, &detected)) {
      RCLCPP_WARN(
        node_->get_logger(), "[Phase2] iter %d: perception failed, skipping.", iter + 1);
      continue;
    }

    const Eigen::VectorXd q_now = current_q_snapshot();
    if (q_now.size() != 5) {
      RCLCPP_ERROR(node_->get_logger(), "[Phase2] No valid joint state.");
      return false;
    }
    const auto fk = kin_engine_->forwardKinematics(q_now);
    const double ex = fk.translation().x();
    const double ey = fk.translation().y();

    const double dx = detected.position.x - ex;
    const double dy = detected.position.y - ey;
    const double error = std::hypot(dx, dy);

    RCLCPP_INFO(
      node_->get_logger(),
      "[Phase2] iter %d/%d | object=(%.3f,%.3f) EE=(%.3f,%.3f) err=%.4f m",
      iter + 1, config_.align_max_iters,
      detected.position.x, detected.position.y, ex, ey, error);

    if (error < config_.align_threshold) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[Phase2] Converged (%.4f m < %.4f m threshold).",
        error, config_.align_threshold);
      target_world.position.x = detected.position.x;
      target_world.position.y = detected.position.y;
      return true;
    }

    target_world.position.x = detected.position.x;
    target_world.position.y = detected.position.y;

    Eigen::VectorXd q_new = q_overhead;
    q_new[0] = std::atan2(detected.position.y, detected.position.x);
    q_new[4] = 0.0;
    if (!motion_client_->send_move_goal({q_new})) {
      return false;
    }
    if (!motion_client_->wait_for_action_completion()) {
      return false;
    }
    primitives_->wait_joints_still(0.02, 600);
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "[Phase2] Max iters (%d) reached without convergence. Proceeding with last estimate.",
    config_.align_max_iters);
  return true;
}

bool ThreePhasePipeline::phase3_grasp_descend(const geometry_msgs::msg::Pose & target_world)
{
  RCLCPP_INFO(node_->get_logger(), "[Phase3] Grasp: rotating joint_4 to -90deg...");

  Eigen::VectorXd q_rot = current_q_snapshot();
  if (q_rot.size() == 5) {
    q_rot[4] = -M_PI / 2.0;
    if (!motion_client_->send_move_goal({q_rot})) {
      return false;
    }
    if (!motion_client_->wait_for_action_completion()) {
      return false;
    }
    primitives_->wait_joints_still(0.02, 600);
  }

  RCLCPP_INFO(node_->get_logger(), "[Phase3] Joint_4 at -90deg. Confirm to descend:");
  std::cout << "\n[Phase3/PreGrasp-confirm] Continue: (r)eal / (m)anual-confirm / (a)bort: ";
  char c = 'r';
  std::cin >> c;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  if (c == 'a') {
    RCLCPP_WARN(node_->get_logger(), "[Phase3] Aborted before grasp.");
    return false;
  }

  if (!primitives_->do_grasp_move(target_world)) {
    return false;
  }
  primitives_->do_suction_on();
  return true;
}

bool ThreePhasePipeline::phase3_place_descend(const geometry_msgs::msg::Pose & target_world)
{
  RCLCPP_INFO(node_->get_logger(), "[Phase3P] Place: rotating joint_4 to -90deg...");

  Eigen::VectorXd q_rot = current_q_snapshot();
  if (q_rot.size() == 5) {
    q_rot[4] = -M_PI / 2.0;
    if (!motion_client_->send_move_goal({q_rot})) {
      return false;
    }
    if (!motion_client_->wait_for_action_completion()) {
      return false;
    }
    primitives_->wait_joints_still(0.02, 600);
  }

  std::cout << "\n[Phase3P/PrePlace-confirm] Continue: (r)eal / (m)anual-confirm / (a)bort: ";
  char c = 'r';
  std::cin >> c;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  if (c == 'a') {
    RCLCPP_WARN(node_->get_logger(), "[Phase3P] Aborted before place.");
    return false;
  }

  if (!primitives_->do_place_move(target_world)) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[Phase3P] suction OFF");
  rclcpp::sleep_for(200ms);
  primitives_->do_suction_off();
  rclcpp::sleep_for(300ms);

  RCLCPP_INFO(node_->get_logger(), "[Phase3P] Place done.");
  return true;
}

bool ThreePhasePipeline::do_grasp()
{
  RCLCPP_INFO(node_->get_logger(), "=== 3-Phase Grasp Pipeline ===");

  geometry_msgs::msg::Pose coarse_target;
  if (!phase1_get_coarse_target(coarse_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[3PGrasp] Phase1 failed.");
    return false;
  }
  if (!phase2_align(coarse_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[3PGrasp] Phase2 alignment failed.");
    return false;
  }

  target_pub_->publish(coarse_target);
  if (!phase3_grasp_descend(coarse_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[3PGrasp] Phase3 grasp failed.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "=== 3-Phase Grasp Complete ===");
  return true;
}

bool ThreePhasePipeline::do_place()
{
  RCLCPP_INFO(node_->get_logger(), "=== 3-Phase Place Pipeline ===");

  geometry_msgs::msg::Pose coarse_target;
  if (!phase1_get_coarse_target(coarse_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[3PPlace] Phase1 failed.");
    return false;
  }
  if (!phase2_align(coarse_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[3PPlace] Phase2 alignment failed.");
    return false;
  }

  target_pub_->publish(coarse_target);
  if (!phase3_place_descend(coarse_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[3PPlace] Phase3 place failed.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "=== 3-Phase Place Complete ===");
  return true;
}

}  // namespace arm2_task::task
