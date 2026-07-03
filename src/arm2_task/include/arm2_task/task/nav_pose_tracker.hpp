#pragma once

#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include "navigation/msg/map_point_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace arm2_task::task
{

struct PlanarPose
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PlanarTransform
{
  bool enabled{true};
  bool prefer_tf{true};
  std::string parent_frame{"base_link"};
  std::string child_frame{"lidar_link"};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RelativePlanarPose
{
  PlanarPose lidar_pose;
  PlanarPose arm_pose;
  PlanarPose robot_pose;
  PlanarPose task_pose;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
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
    std::string task_points_topic{"/navigation/task_points"};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    PlanarTransform lidar_in_arm;
    std::vector<PlanarPose> task_points;
  };

  NavPoseTracker(rclcpp::Node * node, Config config);

  bool get_lidar_pose(PlanarPose * pose) const;
  bool get_arm_pose(PlanarPose * pose) const;
  bool get_robot_pose(PlanarPose * pose) const;
  bool get_task_point_pose(int task_id, PlanarPose * pose) const;
  bool has_received_task_points() const;
  bool get_nearest_task_point(PlanarPose * pose, double * distance) const;
  bool compute_relative_pose(int task_id, RelativePlanarPose * pose) const;
  bool compute_relative_pose_to_task_pose(
    const PlanarPose & task_pose,
    RelativePlanarPose * pose) const;
  bool compute_relative_pose_to_nearest(RelativePlanarPose * pose) const;

private:
  static double yaw_from_quaternion_wxyz(double w, double x, double y, double z);
  static PlanarPose arm_pose_from_lidar_pose(
    const PlanarPose & lidar_pose,
    const PlanarTransform & lidar_in_arm);

  PlanarTransform resolve_lidar_in_arm_transform();

  rclcpp::Node * node_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  PlanarTransform lidar_in_arm_;
  bool warned_tf_lookup_failure_{false};
  bool logged_tf_lookup_success_{false};
  mutable std::mutex task_points_mutex_;
  bool has_received_task_points_{false};
  std::vector<PlanarPose> task_points_;
  mutable std::mutex robot_pose_mutex_;
  bool has_robot_pose_{false};
  PlanarPose lidar_pose_;
  PlanarPose arm_pose_;
  PlanarPose robot_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<navigation::msg::MapPointArray>::SharedPtr task_points_sub_;
};

}  // namespace arm2_task::task
