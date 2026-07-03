#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm2_task::task
{

struct PlanarPose
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RelativePlanarPose
{
  PlanarPose robot_pose;
  PlanarPose task_pose;
  double dx_world{0.0};
  double dy_world{0.0};
  double dx_body{0.0};
  double dy_body{0.0};
  double dyaw{0.0};
  double distance{0.0};
};

class NavPoseTracker
{
public:
  struct Config
  {
    std::string state_topic{"/navigation/state"};
    std::vector<PlanarPose> task_points;
  };

  NavPoseTracker(rclcpp::Node * node, Config config);

  bool get_robot_pose(PlanarPose * pose) const;
  bool get_task_point_pose(int task_id, PlanarPose * pose) const;
  bool get_nearest_task_point(PlanarPose * pose, double * distance) const;
  bool compute_relative_pose(int task_id, RelativePlanarPose * pose) const;
  bool compute_relative_pose_to_nearest(RelativePlanarPose * pose) const;

private:
  static double yaw_from_quaternion_wxyz(double w, double x, double y, double z);

  rclcpp::Node * node_{nullptr};
  std::vector<PlanarPose> task_points_;
  mutable std::mutex robot_pose_mutex_;
  bool has_robot_pose_{false};
  PlanarPose robot_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
};

}  // namespace arm2_task::task
