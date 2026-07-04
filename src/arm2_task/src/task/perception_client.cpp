#include "arm2_task/task/perception_client.hpp"

#include <future>
#include <sstream>
#include <utility>

#include "arm2_task/task/timing_events.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace arm2_task::task
{

PerceptionClient::PerceptionClient(
  rclcpp::Node * node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client,
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr place_client,
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr stack_client)
: node_(node),
  tf_buffer_(std::move(tf_buffer)),
  pick_client_(std::move(pick_client)),
  place_client_(std::move(place_client)),
  stack_client_(std::move(stack_client))
{
  timing_event_pub_ = create_timing_event_publisher(node_);
}

bool PerceptionClient::transform_pose_stamped_to_world(
  const geometry_msgs::msg::PoseStamped & stamped_pose,
  const std::string & service_name,
  geometry_msgs::msg::Pose * out_pose)
{
  publish_timing_event(
    node_, timing_event_pub_, "tf", service_name + "_to_world", "begin",
    "frame=" + stamped_pose.header.frame_id);

  const std::string frame_id = stamped_pose.header.frame_id;
  if (frame_id.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "%s returned empty frame_id", service_name.c_str());
    publish_timing_event(
      node_, timing_event_pub_, "tf", service_name + "_to_world", "end",
      "ok=0,reason=empty_frame");
    return false;
  }

  try {
    const geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
      "world", frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));

    if (out_pose != nullptr) {
      geometry_msgs::msg::PoseStamped pose_world;
      tf2::doTransform(stamped_pose, pose_world, t_stamped);
      *out_pose = pose_world.pose;
      RCLCPP_INFO(
        node_->get_logger(),
        "%s frame=%s -> world=(%.3f, %.3f, %.3f)",
        service_name.c_str(), frame_id.c_str(),
        out_pose->position.x, out_pose->position.y, out_pose->position.z);
    }
    publish_timing_event(
      node_, timing_event_pub_, "tf", service_name + "_to_world", "end",
      "frame=" + frame_id + ",ok=1");
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "TF2 error in %s: %s", service_name.c_str(), ex.what());
    publish_timing_event(
      node_, timing_event_pub_, "tf", service_name + "_to_world", "end",
      "frame=" + frame_id + ",ok=0,reason=tf_error");
    return false;
  }
}

bool PerceptionClient::call_pick_service_sync(
  const std::string & object_name,
  geometry_msgs::msg::Pose * out_pose)
{
  const std::string detail = "object=" + object_name;
  publish_timing_event(node_, timing_event_pub_, "perception", "get_pick_pos", "begin", detail);

  if (!pick_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(), "get_pick_pos service not available");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_pick_pos", "end",
      detail + ",ok=0,reason=service_unavailable");
    return false;
  }

  auto request = std::make_shared<robot_msgs::srv::GetPickPos::Request>();
  request->object_name = object_name;

  auto result_future = pick_client_->async_send_request(request);
  if (result_future.wait_for(15s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "get_pick_pos service call timed out (waited 15 seconds)");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_pick_pos", "end",
      detail + ",ok=0,reason=timeout");
    return false;
  }

  const auto response = result_future.get();
  if (!response) {
    RCLCPP_ERROR(node_->get_logger(), "get_pick_pos service call returned null response");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_pick_pos", "end",
      detail + ",ok=0,reason=null_response");
    return false;
  }
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "get_pick_pos service returned failure (success=false)");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_pick_pos", "end",
      detail + ",ok=0,reason=response_failure");
    return false;
  }

  const bool ok = transform_pose_stamped_to_world(response->pick_pose, "get_pick_pos", out_pose);
  publish_timing_event(
    node_, timing_event_pub_, "perception", "get_pick_pos", "end",
    detail + ",ok=" + timing_bool(ok));
  return ok;
}

bool PerceptionClient::call_place_service_sync(
  const std::string & frame_name,
  geometry_msgs::msg::Pose * out_pose)
{
  const std::string detail = "frame=" + frame_name;
  publish_timing_event(node_, timing_event_pub_, "perception", "get_place_pos", "begin", detail);

  if (!place_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(), "get_place_pos service not available");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_place_pos", "end",
      detail + ",ok=0,reason=service_unavailable");
    return false;
  }

  auto request = std::make_shared<robot_msgs::srv::GetPlacePos::Request>();
  request->frame_name = frame_name;

  auto result_future = place_client_->async_send_request(request);
  if (result_future.wait_for(15s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "get_place_pos service call timed out (waited 15 seconds)");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_place_pos", "end",
      detail + ",ok=0,reason=timeout");
    return false;
  }

  const auto response = result_future.get();
  if (!response) {
    RCLCPP_ERROR(node_->get_logger(), "get_place_pos service call returned null response");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_place_pos", "end",
      detail + ",ok=0,reason=null_response");
    return false;
  }
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "get_place_pos service returned failure (success=false)");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_place_pos", "end",
      detail + ",ok=0,reason=response_failure");
    return false;
  }

  const bool ok = transform_pose_stamped_to_world(response->place_pose, "get_place_pos", out_pose);
  publish_timing_event(
    node_, timing_event_pub_, "perception", "get_place_pos", "end",
    detail + ",ok=" + timing_bool(ok));
  return ok;
}

bool PerceptionClient::call_stack_service_sync(
  const std::string & frame_name,
  geometry_msgs::msg::Pose * out_pose)
{
  const std::string detail = "frame=" + frame_name;
  publish_timing_event(node_, timing_event_pub_, "perception", "get_stack_pos", "begin", detail);

  if (!stack_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(), "get_stack_pos service not available");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_stack_pos", "end",
      detail + ",ok=0,reason=service_unavailable");
    return false;
  }

  auto request = std::make_shared<robot_msgs::srv::GetPlacePos::Request>();
  request->frame_name = frame_name;

  auto result_future = stack_client_->async_send_request(request);
  if (result_future.wait_for(15s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "get_stack_pos service call timed out");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_stack_pos", "end",
      detail + ",ok=0,reason=timeout");
    return false;
  }

  const auto response = result_future.get();
  if (!response || !response->success) {
    RCLCPP_ERROR(node_->get_logger(), "get_stack_pos service returned failure");
    publish_timing_event(
      node_, timing_event_pub_, "perception", "get_stack_pos", "end",
      detail + ",ok=0,reason=response_failure");
    return false;
  }

  const bool ok = transform_pose_stamped_to_world(response->place_pose, "get_stack_pos", out_pose);
  publish_timing_event(
    node_, timing_event_pub_, "perception", "get_stack_pos", "end",
    detail + ",ok=" + timing_bool(ok));
  return ok;
}

}  // namespace arm2_task::task
