#include "arm2_task/task/nav_pose_tracker.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "arm2_task/task/pose_utils.hpp"

namespace arm2_task::task
{
namespace
{

RelativePlanarPose build_relative_pose(
  const PlanarPose & robot_pose,
  const PlanarPose & task_pose)
{
  RelativePlanarPose relative;
  relative.robot_pose = robot_pose;
  relative.task_pose = task_pose;
  relative.dx_world = task_pose.x - robot_pose.x;
  relative.dy_world = task_pose.y - robot_pose.y;
  relative.distance = std::hypot(relative.dx_world, relative.dy_world);

  const double cos_yaw = std::cos(robot_pose.yaw);
  const double sin_yaw = std::sin(robot_pose.yaw);
  relative.dx_body = cos_yaw * relative.dx_world + sin_yaw * relative.dy_world;
  relative.dy_body = -sin_yaw * relative.dx_world + cos_yaw * relative.dy_world;
  relative.dyaw = normalize_angle(task_pose.yaw - robot_pose.yaw);
  return relative;
}

}  // namespace

NavPoseTracker::NavPoseTracker(rclcpp::Node * node, Config config)
: node_(node),
  task_points_(std::move(config.task_points))
{
  const auto topic = config.state_topic.empty() ? "/navigation/state" : config.state_topic;
  state_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic,
    rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (!msg) {
        return;
      }

      PlanarPose next;
      next.x = msg->pose.pose.position.x;
      next.y = msg->pose.pose.position.y;
      next.yaw = yaw_from_quaternion_wxyz(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

      std::lock_guard<std::mutex> lock(robot_pose_mutex_);
      robot_pose_ = next;
      has_robot_pose_ = true;
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav_pose] Tracking %zu static task points from %s.",
    task_points_.size(), topic.c_str());
}

bool NavPoseTracker::get_robot_pose(PlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(robot_pose_mutex_);
  if (!has_robot_pose_) {
    return false;
  }

  *pose = robot_pose_;
  return true;
}

bool NavPoseTracker::get_task_point_pose(int task_id, PlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  for (const auto & point : task_points_) {
    if (point.id == task_id) {
      *pose = point;
      return true;
    }
  }
  return false;
}

bool NavPoseTracker::get_nearest_task_point(PlanarPose * pose, double * distance) const
{
  if (pose == nullptr || distance == nullptr || task_points_.empty()) {
    return false;
  }

  PlanarPose robot_pose;
  if (!get_robot_pose(&robot_pose)) {
    return false;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  const PlanarPose * best_point = nullptr;
  for (const auto & point : task_points_) {
    const double dx = point.x - robot_pose.x;
    const double dy = point.y - robot_pose.y;
    const double current_distance = std::hypot(dx, dy);
    if (current_distance < best_distance) {
      best_distance = current_distance;
      best_point = &point;
    }
  }

  if (best_point == nullptr) {
    return false;
  }

  *pose = *best_point;
  *distance = best_distance;
  return true;
}

bool NavPoseTracker::compute_relative_pose(int task_id, RelativePlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  PlanarPose robot_pose;
  PlanarPose task_pose;
  if (!get_robot_pose(&robot_pose) || !get_task_point_pose(task_id, &task_pose)) {
    return false;
  }

  *pose = build_relative_pose(robot_pose, task_pose);
  return true;
}

bool NavPoseTracker::compute_relative_pose_to_nearest(RelativePlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  PlanarPose robot_pose;
  PlanarPose task_pose;
  double distance = 0.0;
  if (!get_robot_pose(&robot_pose) || !get_nearest_task_point(&task_pose, &distance)) {
    return false;
  }

  *pose = build_relative_pose(robot_pose, task_pose);
  pose->distance = distance;
  return true;
}

double NavPoseTracker::yaw_from_quaternion_wxyz(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace arm2_task::task
