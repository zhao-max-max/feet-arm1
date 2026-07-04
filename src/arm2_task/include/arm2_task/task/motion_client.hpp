#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_joint.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "std_msgs/msg/string.hpp"

namespace arm2_task::task
{

class MotionClient
{
public:
  using MoveJoint = robot_msgs::action::MoveJoint;
  using GoalHandleMoveJoint = rclcpp_action::ClientGoalHandle<MoveJoint>;

  MotionClient(
    rclcpp::Node * node,
    rclcpp_action::Client<MoveJoint>::SharedPtr move_joint_client,
    rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client,
    const std::atomic<bool> * is_running);

  void set_trajectory_defaults(double max_velocity, double max_acceleration, double blend_radius);

  bool send_move_goal(const std::vector<Eigen::VectorXd> & q_waypoints);

  bool send_move_goal(const Eigen::VectorXd & q_single);

  bool wait_for_action_completion(std::chrono::seconds timeout = std::chrono::seconds(30));

  int request_mode_switch(const std::string & mode_name);

private:
  bool task_is_running() const;

  rclcpp::Node * node_{nullptr};
  rclcpp_action::Client<MoveJoint>::SharedPtr move_joint_client_;
  rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
  const std::atomic<bool> * is_running_{nullptr};

  std::atomic<bool> is_action_running_{false};
  std::atomic<bool> action_finished_{false};
  bool last_action_succeeded_{false};
  std::string last_action_message_;
  std::mutex action_result_mutex_;

  double max_v_{1.0};
  double max_a_{2.0};
  double blend_radius_{0.05};
  int move_goal_sequence_{0};
  int active_move_goal_sequence_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_event_pub_;
};

}  // namespace arm2_task::task
