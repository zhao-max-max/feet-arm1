#pragma once

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace arm2_task::task
{

inline rclcpp::Publisher<std_msgs::msg::String>::SharedPtr create_timing_event_publisher(
  rclcpp::Node * node)
{
  if (node == nullptr) {
    return nullptr;
  }
  return node->create_publisher<std_msgs::msg::String>(
    "/debug/arm_task_timing", rclcpp::QoS(rclcpp::KeepLast(200)).reliable());
}

inline void publish_timing_event(
  rclcpp::Node * node,
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
  const std::string & kind,
  const std::string & name,
  const std::string & phase,
  const std::string & detail = "")
{
  if (node == nullptr || !publisher) {
    return;
  }

  std_msgs::msg::String msg;
  std::ostringstream out;
  out << std::fixed << std::setprecision(9)
      << "source_ros_time=" << node->now().seconds()
      << "|source_node=" << node->get_name()
      << "|kind=" << kind
      << "|name=" << name
      << "|phase=" << phase;
  if (!detail.empty()) {
    out << "|detail=" << detail;
  }
  msg.data = out.str();
  publisher->publish(msg);
}

inline const char * timing_bool(bool value)
{
  return value ? "1" : "0";
}

}  // namespace arm2_task::task
