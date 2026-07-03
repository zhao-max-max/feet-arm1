#include "arm2_task/task/end_effector_client.hpp"

#include <future>
#include <utility>

using namespace std::chrono_literals;

namespace arm2_task::task
{

EndEffectorClient::EndEffectorClient(
  rclcpp::Node * node,
  rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client,
  rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_client,
  rclcpp::Client<robot_msgs::srv::SetPayloadState>::SharedPtr payload_state_client)
: node_(node),
  suction_client_(std::move(suction_client)),
  payload_client_(std::move(payload_client)),
  payload_state_client_(std::move(payload_state_client))
{
}

int EndEffectorClient::set_suction(bool activate, bool required)
{
  if (!suction_client_->wait_for_service(1s)) {
    if (required) {
      RCLCPP_ERROR(node_->get_logger(), "Required suction service is not available.");
      return 0;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Optional suction service unavailable; skipping suction command.");
    return 1;
  }

  auto request = std::make_shared<robot_msgs::srv::SetSuction::Request>();
  request->activate = activate;

  auto result_future = suction_client_->async_send_request(request);
  if (result_future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(
      node_->get_logger(), "Suction command [%s] timed out.", activate ? "ON" : "OFF");
    return 0;
  }

  const auto response = result_future.get();
  if (!response || !response->success) {
    RCLCPP_ERROR(
      node_->get_logger(), "Suction command [%s] failed.", activate ? "ON" : "OFF");
    return 0;
  }

  RCLCPP_INFO(node_->get_logger(), "Suction %s", activate ? "ON" : "OFF");
  return 1;
}

bool EndEffectorClient::request_payload_estimate(double * out_mass)
{
  if (!payload_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(), "Service [GetPayloadEstimate] not available.");
    return false;
  }

  auto request = std::make_shared<robot_msgs::srv::GetPayloadEstimate::Request>();
  auto result_future = payload_client_->async_send_request(request);
  if (result_future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Payload estimate service call timed out.");
    return false;
  }

  const auto response = result_future.get();
  if (!response || !response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Payload estimate service responded with failure.");
    return false;
  }

  if (out_mass != nullptr) {
    *out_mass = static_cast<double>(response->mass);
  }
  return true;
}

int EndEffectorClient::request_payload_state(
  bool has_load,
  bool required,
  bool default_has_load,
  double default_mass,
  const std::vector<double> & default_com)
{
  if (!payload_state_client_->wait_for_service(1s)) {
    if (required) {
      RCLCPP_ERROR(node_->get_logger(), "Required payload state service is not available.");
      return 0;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Optional payload state service unavailable; skipping payload model update.");
    return 1;
  }

  auto request = std::make_shared<robot_msgs::srv::SetPayloadState::Request>();
  request->has_load = has_load ? default_has_load : false;
  request->mass = default_mass;
  for (size_t i = 0; i < 3; ++i) {
    request->com[i] = default_com[i];
  }

  auto result_future = payload_state_client_->async_send_request(request);
  if (result_future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Payload state update timed out.");
    return 0;
  }

  const auto response = result_future.get();
  if (!response || !response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Payload state update failed.");
    return 0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Payload model %s: mass=%.3f com=[%.3f, %.3f, %.3f]",
    has_load ? "enabled" : "cleared",
    request->mass,
    request->com[0],
    request->com[1],
    request->com[2]);
  return 1;
}

}  // namespace arm2_task::task
