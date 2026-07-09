#include "arm2_task/task/task_sequences.hpp"

#include <cmath>
#include <utility>

#include "arm2_task/task/timing_events.hpp"

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
  timing_event_pub_ = create_timing_event_publisher(node_);
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
  publish_timing_event(node_, timing_event_pub_, "sequence", "grasp_from_perception", "begin");
  geometry_msgs::msg::Pose target;

  motion_client_->request_mode_switch("moving");
  primitives_->do_look_out(make_forward_pose());
  primitives_->wait_joints_still(0.02, 200);

  if (!perception_client_->call_pick_service_sync(config_.pick_object_name, &target)) {
    RCLCPP_ERROR(node_->get_logger(), "[grasp] Perception failed.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "grasp_from_perception", "end",
      "ok=0,reason=perception_failed");
    return false;
  }

  const bool ok = grasp_pose(target, true);
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "grasp_from_perception", "end",
    std::string("ok=") + timing_bool(ok));
  return ok;
}

bool TaskSequences::grasp_from_current_view()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "task_grasp_current_view", "begin");
  const bool ok = primitives_->do_grasp_from_current_view();
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "task_grasp_current_view", "end",
    std::string("ok=") + timing_bool(ok));
  return ok;
}

bool TaskSequences::grasp_mock_or_perception()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "grasp_mock_or_perception", "begin");
  if (!config_.use_mock_grasp_target) {
    const bool ok = grasp_from_perception();
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "grasp_mock_or_perception", "end",
      std::string("ok=") + timing_bool(ok) + ",source=perception");
    return ok;
  }

  geometry_msgs::msg::Pose target;
  target.position.x = config_.grasp_mock_x;
  target.position.y = config_.grasp_mock_y;
  target.position.z = config_.grasp_mock_z;
  target.orientation.w = 1.0;
  RCLCPP_INFO(
    node_->get_logger(), "Using mock target (%.3f, %.3f, %.3f)",
    config_.grasp_mock_x, config_.grasp_mock_y, config_.grasp_mock_z);
  const bool ok = grasp_pose(target, true);
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "grasp_mock_or_perception", "end",
    std::string("ok=") + timing_bool(ok) + ",source=mock");
  return ok;
}

bool TaskSequences::grasp_pose(const geometry_msgs::msg::Pose & target, bool aligned)
{
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "grasp_pose", "begin",
    std::string("aligned=") + timing_bool(aligned));
  if (aligned) {
    if (!primitives_->do_full_grasp_aligned(target)) {
      RCLCPP_ERROR(node_->get_logger(), "[grasp] Grasp failed.");
      publish_timing_event(
        node_, timing_event_pub_, "sequence", "grasp_pose", "end",
        "ok=0,aligned=1");
      return false;
    }
  } else if (!primitives_->do_full_grasp(target)) {
    RCLCPP_ERROR(node_->get_logger(), "[grasp] Grasp failed.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "grasp_pose", "end",
      "ok=0,aligned=0");
    return false;
  }
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "grasp_pose", "end",
    std::string("ok=1,aligned=") + timing_bool(aligned));
  return true;
}

bool TaskSequences::place_from_perception()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "place_from_perception", "begin");
  geometry_msgs::msg::Pose frame_pose;
  if (!perception_client_->call_place_service_sync(config_.place_frame_name, &frame_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[place] Place perception failed.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "place_from_perception", "end",
      "ok=0,reason=perception_failed");
    return false;
  }
  const bool ok = place_pose(frame_pose);
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "place_from_perception", "end",
    std::string("ok=") + timing_bool(ok));
  return ok;
}

bool TaskSequences::place_mock_or_perception()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "place_mock_or_perception", "begin");
  geometry_msgs::msg::Pose frame_pose;
  std::string source = "perception";
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
    source = "mock";
  } else if (!perception_client_->call_place_service_sync(config_.place_frame_name, &frame_pose)) {
    RCLCPP_WARN(node_->get_logger(), "[place] Perception failed, abort.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "place_mock_or_perception", "end",
      "ok=0,reason=perception_failed");
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "[place] frame world=(%.3f,%.3f,%.3f)",
    frame_pose.position.x, frame_pose.position.y, frame_pose.position.z);
  const bool ok = place_pose(frame_pose);
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "place_mock_or_perception", "end",
    std::string("ok=") + timing_bool(ok) + ",source=" + source);
  return ok;
}

bool TaskSequences::place_pose(const geometry_msgs::msg::Pose & frame_pose)
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "place_pose", "begin");
  motion_client_->request_mode_switch("moving");
  if (!primitives_->do_place_move_with_orientation(frame_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[place] Place move failed.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "place_pose", "end",
      "ok=0,reason=place_move_failed");
    return false;
  }
  motion_client_->request_mode_switch("moving");
  publish_timing_event(node_, timing_event_pub_, "sequence", "place_pose", "end", "ok=1");
  return true;
}

bool TaskSequences::place_pose_direct_height(const geometry_msgs::msg::Pose & target_pose)
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "place_pose_direct_height", "begin");
  motion_client_->request_mode_switch("moving");
  if (!primitives_->do_place_move_with_direct_height(target_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[place_direct] Place move failed.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "place_pose_direct_height", "end",
      "ok=0,reason=place_move_failed");
    return false;
  }
  motion_client_->request_mode_switch("moving");
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "place_pose_direct_height", "end", "ok=1");
  return true;
}

bool TaskSequences::stack_mock_or_perception()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "stack_mock_or_perception", "begin");
  geometry_msgs::msg::Pose box_top_pose;
  std::string source = "perception";
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
    source = "mock";
  } else if (!perception_client_->call_stack_service_sync(config_.stack_service_name, &box_top_pose)) {
    RCLCPP_WARN(node_->get_logger(), "[stack] Stack perception failed, abort.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "stack_mock_or_perception", "end",
      "ok=0,reason=perception_failed");
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "[stack] box_top world=(%.3f,%.3f,%.3f)",
    box_top_pose.position.x, box_top_pose.position.y, box_top_pose.position.z);
  const bool ok = stack_pose(box_top_pose);
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "stack_mock_or_perception", "end",
    std::string("ok=") + timing_bool(ok) + ",source=" + source);
  return ok;
}

bool TaskSequences::stack_pose(const geometry_msgs::msg::Pose & box_top_pose)
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "stack_pose", "begin");
  motion_client_->request_mode_switch("moving");
  if (!primitives_->do_stack_move_with_orientation(box_top_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[stack] Stack move failed.");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "stack_pose", "end",
      "ok=0,reason=stack_move_failed");
    return false;
  }
  motion_client_->request_mode_switch("moving");
  publish_timing_event(node_, timing_event_pub_, "sequence", "stack_pose", "end", "ok=1");
  return true;
}

bool TaskSequences::move_to_carry_loaded()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "move_to_carry_loaded", "begin");
  RCLCPP_INFO(node_->get_logger(), "[carry] moving -> carry preset -> loaded");
  if (!motion_client_->request_mode_switch("moving")) {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "move_to_carry_loaded", "end",
      "ok=0,reason=mode_moving_failed");
    return false;
  }
  if (!presets_->count("carry")) {
    RCLCPP_ERROR(node_->get_logger(), "[carry] Preset 'carry' not found!");
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "move_to_carry_loaded", "end",
      "ok=0,reason=missing_preset");
    return false;
  }
  if (!motion_client_->send_move_goal({presets_->at("carry")}) ||
    !motion_client_->wait_for_action_completion())
  {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "move_to_carry_loaded", "end",
      "ok=0,reason=move_failed");
    return false;
  }
  motion_client_->request_mode_switch("loaded");
  publish_timing_event(
    node_, timing_event_pub_, "sequence", "move_to_carry_loaded", "end", "ok=1");
  return true;
}

bool TaskSequences::store_to_dog()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "store_to_dog", "begin");

  // Grasp box (arm suction ON, loaded mode)
  if (!primitives_->do_grasp_from_current_view()) {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "store_to_dog", "end",
      "ok=0,reason=grasp_failed");
    return false;
  }

  // Move to store preset (hand-off position above dog suction cup)
  primitives_->do_store_pose();

  // Hand off: arm suction OFF (also switches to moving mode), then dog suction ON
  primitives_->do_suction_off();
  primitives_->do_dog_suction_on();

  // Return to reset
  primitives_->do_reset();

  publish_timing_event(
    node_, timing_event_pub_, "sequence", "store_to_dog", "end", "ok=1");
  return true;
}

bool TaskSequences::pickup_from_dog_and_place()
{
  publish_timing_event(node_, timing_event_pub_, "sequence", "pickup_from_dog_and_place", "begin");

  // Move to store preset to reach box on dog suction cup
  primitives_->do_store_pose();

  // Release dog suction, then arm takes the box
  primitives_->do_dog_suction_off();
  primitives_->do_suction_on();  // 400ms settle + arm suction ON + loaded mode

  // Place the box using perception (same as normal place sequence)
  const bool place_ok = place_from_perception();
  if (!place_ok) {
    publish_timing_event(
      node_, timing_event_pub_, "sequence", "pickup_from_dog_and_place", "end",
      "ok=0,reason=place_failed");
    // Still reset and re-enable dog suction even on failure
    primitives_->do_reset();
    primitives_->do_dog_suction_on();
    return false;
  }

  primitives_->do_reset();
  // Re-enable dog suction (ready for next cycle)
  primitives_->do_dog_suction_on();

  publish_timing_event(
    node_, timing_event_pub_, "sequence", "pickup_from_dog_and_place", "end", "ok=1");
  return true;
}

}  // namespace arm2_task::task
