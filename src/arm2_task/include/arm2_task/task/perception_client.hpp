#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/get_pick_pos.hpp"
#include "robot_msgs/srv/get_place_pos.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace arm2_task::task
{

class PerceptionClient
{
public:
  PerceptionClient(
    rclcpp::Node * node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client,
    rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr place_client,
    rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr stack_client);

  bool call_pick_service_sync(
    const std::string & object_name,
    geometry_msgs::msg::Pose * out_pose);

  bool call_place_service_sync(
    const std::string & frame_name,
    geometry_msgs::msg::Pose * out_pose);

  bool call_stack_service_sync(
    const std::string & frame_name,
    geometry_msgs::msg::Pose * out_pose);

private:
  bool transform_pose_stamped_to_world(
    const geometry_msgs::msg::PoseStamped & stamped_pose,
    const std::string & service_name,
    geometry_msgs::msg::Pose * out_pose);

  rclcpp::Node * node_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr place_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr stack_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_event_pub_;
};

}  // namespace arm2_task::task
