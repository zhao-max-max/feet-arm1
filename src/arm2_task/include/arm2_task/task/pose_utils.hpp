#pragma once

#include "geometry_msgs/msg/pose.hpp"

namespace arm2_task::task
{

struct EdgeAlignedRoll
{
  bool identity_orientation{false};
  double alpha{0.0};
  double chosen{0.0};
  double edge_yaw{0.0};
  double base_yaw{0.0};
  double roll{0.0};
};

double normalize_angle(double angle);

double object_yaw_roll(const geometry_msgs::msg::Pose & object_world);

EdgeAlignedRoll compute_edge_aligned_roll(const geometry_msgs::msg::Pose & world_pose);

double compute_stack_tool_roll(
  const geometry_msgs::msg::Pose & box_top_world,
  double roll_sign);

}  // namespace arm2_task::task
