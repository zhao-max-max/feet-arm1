#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/get_payload_estimate.hpp"
#include "robot_msgs/srv/set_payload_state.hpp"
#include "robot_msgs/srv/set_suction.hpp"

namespace arm2_task::task
{

class EndEffectorClient
{
public:
  EndEffectorClient(
    rclcpp::Node * node,
    rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client,
    rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr dog_suction_client,
    rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_client,
    rclcpp::Client<robot_msgs::srv::SetPayloadState>::SharedPtr payload_state_client);

  int set_suction(bool activate, bool required);
  int set_dog_suction(bool activate, bool required);

  bool request_payload_estimate(double * out_mass);

  int request_payload_state(
    bool has_load,
    bool required,
    bool default_has_load,
    double default_mass,
    const std::vector<double> & default_com);

private:
  rclcpp::Node * node_{nullptr};
  rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client_;
  rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr dog_suction_client_;
  rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_client_;
  rclcpp::Client<robot_msgs::srv::SetPayloadState>::SharedPtr payload_state_client_;
};

}  // namespace arm2_task::task
