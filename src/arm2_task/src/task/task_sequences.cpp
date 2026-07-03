#include "arm2_task/task/task_sequences.hpp"

#include <cmath>
#include <utility>

namespace arm2_task::task
{

TaskSequences::TaskSequences(
  rclcpp::Node * node,
  MotionClient * motion_client,
  PerceptionClient * perception_client,
  TaskPrimitives * primitives,
  const std::map<std::string, Eigen::VectorXd> * presets,
  Config config)
: node_(node),
  motion_client_(motion_client),
  perception_client_(perception_client),
  primitives_(primitives),
  presets_(presets),
  config_(std::move(config))
{
}

geometry_msgs::msg::Pose TaskSequences::make_forward_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Pose TaskSequences::make_yaw_pose(
  double x,
  double y,
  double z,
  double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

bool TaskSequences::grasp_from_perception()
{
  geometry_msgs::msg::Pose target;

  motion_client_->request_mode_switch("moving");
  primitives_->do_look_out(make_forward_pose());
  primitives_->wait_joints_still(0.02, 200);

  if (!perception_client_->call_pick_service_sync(config_.pick_object_name, &target)) {
    RCLCPP_ERROR(node_->get_logger(), "[grasp] Perception failed.");
    return false;
  }

  return grasp_pose(target, true);
}

bool TaskSequences::grasp_from_current_view()
{
  return primitives_->do_grasp_from_current_view();
}

bool TaskSequences::grasp_mock_or_perception()
{
  if (!config_.use_mock_grasp_target) {
    return grasp_from_perception();
  }

  geometry_msgs::msg::Pose target;
  target.position.x = config_.grasp_mock_x;
  target.position.y = config_.grasp_mock_y;
  target.position.z = config_.grasp_mock_z;
  target.orientation.w = 1.0;
  RCLCPP_INFO(
    node_->get_logger(), "Using mock target (%.3f, %.3f, %.3f)",
    config_.grasp_mock_x, config_.grasp_mock_y, config_.grasp_mock_z);
  return grasp_pose(target, true);
}

bool TaskSequences::grasp_pose(const geometry_msgs::msg::Pose & target, bool aligned)
{
  if (aligned) {
    if (!primitives_->do_full_grasp_aligned(target)) {
      RCLCPP_ERROR(node_->get_logger(), "[grasp] Grasp failed.");
      return false;
    }
  } else if (!primitives_->do_full_grasp(target)) {
    RCLCPP_ERROR(node_->get_logger(), "[grasp] Grasp failed.");
    return false;
  }
  return true;
}

bool TaskSequences::place_from_perception()
{
  geometry_msgs::msg::Pose frame_pose;
  if (!perception_client_->call_place_service_sync(config_.place_frame_name, &frame_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[place] Place perception failed.");
    return false;
  }
  return place_pose(frame_pose);
}

bool TaskSequences::place_mock_or_perception()
{
  geometry_msgs::msg::Pose frame_pose;
  if (config_.use_mock_place_frame) {
    frame_pose = make_yaw_pose(
      config_.place_mock_x,
      config_.place_mock_y,
      config_.place_mock_z,
      config_.place_mock_yaw);
    RCLCPP_INFO(
      node_->get_logger(),
      "[place] mock frame pos=(%.3f,%.3f,%.3f) yaw=%.3f rad",
      config_.place_mock_x, config_.place_mock_y,
      config_.place_mock_z, config_.place_mock_yaw);
  } else if (!perception_client_->call_place_service_sync(config_.place_frame_name, &frame_pose)) {
    RCLCPP_WARN(node_->get_logger(), "[place] Perception failed, abort.");
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "[place] frame world=(%.3f,%.3f,%.3f)",
    frame_pose.position.x, frame_pose.position.y, frame_pose.position.z);
  return place_pose(frame_pose);
}

bool TaskSequences::place_pose(const geometry_msgs::msg::Pose & frame_pose)
{
  motion_client_->request_mode_switch("moving");
  if (!primitives_->do_place_move_with_orientation(frame_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[place] Place move failed.");
    return false;
  }
  motion_client_->request_mode_switch("moving");
  return true;
}

bool TaskSequences::stack_mock_or_perception()
{
  geometry_msgs::msg::Pose box_top_pose;
  if (config_.use_mock_stack) {
    box_top_pose = make_yaw_pose(
      config_.stack_mock_x,
      config_.stack_mock_y,
      config_.stack_mock_z,
      config_.stack_mock_yaw);
    RCLCPP_INFO(
      node_->get_logger(),
      "[stack] mock box_top pos=(%.3f,%.3f,%.3f) yaw=%.3f rad",
      config_.stack_mock_x, config_.stack_mock_y,
      config_.stack_mock_z, config_.stack_mock_yaw);
  } else if (!perception_client_->call_stack_service_sync(config_.stack_service_name, &box_top_pose)) {
    RCLCPP_WARN(node_->get_logger(), "[stack] Stack perception failed, abort.");
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "[stack] box_top world=(%.3f,%.3f,%.3f)",
    box_top_pose.position.x, box_top_pose.position.y, box_top_pose.position.z);
  return stack_pose(box_top_pose);
}

bool TaskSequences::stack_pose(const geometry_msgs::msg::Pose & box_top_pose)
{
  motion_client_->request_mode_switch("moving");
  if (!primitives_->do_stack_move_with_orientation(box_top_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[stack] Stack move failed.");
    return false;
  }
  motion_client_->request_mode_switch("moving");
  return true;
}

bool TaskSequences::move_to_carry_loaded()
{
  RCLCPP_INFO(node_->get_logger(), "[carry] moving -> carry preset -> loaded");
  if (!motion_client_->request_mode_switch("moving")) {
    return false;
  }
  if (!presets_->count("carry")) {
    RCLCPP_ERROR(node_->get_logger(), "[carry] Preset 'carry' not found!");
    return false;
  }
  if (!motion_client_->send_move_goal({presets_->at("carry")}) ||
    !motion_client_->wait_for_action_completion())
  {
    return false;
  }
  motion_client_->request_mode_switch("loaded");
  return true;
}

}  // namespace arm2_task::task
