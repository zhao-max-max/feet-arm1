#include "arm2_task/task/nav_pose_tracker.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "arm2_task/task/pose_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace arm2_task::task
{
namespace
{

RelativePlanarPose build_relative_pose(
  const PlanarPose & lidar_pose,
  const PlanarPose & arm_pose,
  const PlanarPose & task_pose)
{
  RelativePlanarPose relative;
  relative.lidar_pose = lidar_pose;
  relative.arm_pose = arm_pose;
  relative.robot_pose = arm_pose;
  relative.task_pose = task_pose;
  relative.dx_world = task_pose.x - arm_pose.x;
  relative.dy_world = task_pose.y - arm_pose.y;
  relative.distance = std::hypot(relative.dx_world, relative.dy_world);

  const double cos_yaw = std::cos(arm_pose.yaw);
  const double sin_yaw = std::sin(arm_pose.yaw);
  relative.dx_body = cos_yaw * relative.dx_world + sin_yaw * relative.dy_world;
  relative.dy_body = -sin_yaw * relative.dx_world + cos_yaw * relative.dy_world;
  relative.dyaw = normalize_angle(task_pose.yaw - arm_pose.yaw);
  relative.x = relative.dx_body;
  relative.y = relative.dy_body;
  relative.yaw = relative.dyaw;
  return relative;
}

}  // namespace

NavPoseTracker::NavPoseTracker(rclcpp::Node * node, Config config)
: node_(node),
  tf_buffer_(std::move(config.tf_buffer)),
  lidar_in_arm_(std::move(config.lidar_in_arm)),
  task_points_(std::move(config.task_points))
{
  const auto topic = config.state_topic.empty() ? "/navigation/state" : config.state_topic;
  const auto task_points_topic = config.task_points_topic.empty() ?
    std::string("/navigation/task_points") : config.task_points_topic;
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

      const auto lidar_in_arm = resolve_lidar_in_arm_transform();
      const auto arm_pose = arm_pose_from_lidar_pose(next, lidar_in_arm);

      std::lock_guard<std::mutex> lock(robot_pose_mutex_);
      lidar_pose_ = next;
      arm_pose_ = arm_pose;
      robot_pose_ = arm_pose;
      has_robot_pose_ = true;
    });

  auto static_points_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  task_points_sub_ = node_->create_subscription<navigation::msg::MapPointArray>(
    task_points_topic,
    static_points_qos,
    [this, task_points_topic](const navigation::msg::MapPointArray::SharedPtr msg) {
      if (!msg) {
        return;
      }

      std::vector<PlanarPose> points;
      points.reserve(msg->points.size());
      for (const auto & map_point : msg->points) {
        PlanarPose point;
        point.id = map_point.id;
        point.x = map_point.x;
        point.y = map_point.y;
        point.yaw = 0.0;
        points.push_back(point);
      }

      {
        std::lock_guard<std::mutex> lock(task_points_mutex_);
        task_points_ = std::move(points);
        has_received_task_points_ = true;
      }

      RCLCPP_INFO(
        node_->get_logger(),
        "[nav_pose] Received %zu task points from %s.",
        msg->points.size(), task_points_topic.c_str());
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav_pose] Tracking %zu fallback task points from %s and subscribing %s; lidar extrinsics %s (%s -> %s).",
    task_points_.size(), topic.c_str(), task_points_topic.c_str(),
    lidar_in_arm_.enabled ? "enabled" : "disabled",
    lidar_in_arm_.parent_frame.c_str(),
    lidar_in_arm_.child_frame.c_str());
}

bool NavPoseTracker::get_lidar_pose(PlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(robot_pose_mutex_);
  if (!has_robot_pose_) {
    return false;
  }

  *pose = lidar_pose_;
  return true;
}

bool NavPoseTracker::get_arm_pose(PlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(robot_pose_mutex_);
  if (!has_robot_pose_) {
    return false;
  }

  *pose = arm_pose_;
  return true;
}

bool NavPoseTracker::get_robot_pose(PlanarPose * pose) const
{
  return get_arm_pose(pose);
}

bool NavPoseTracker::get_task_point_pose(int task_id, PlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(task_points_mutex_);
  for (const auto & point : task_points_) {
    if (point.id == task_id) {
      *pose = point;
      return true;
    }
  }
  return false;
}

bool NavPoseTracker::has_received_task_points() const
{
  std::lock_guard<std::mutex> lock(task_points_mutex_);
  return has_received_task_points_;
}

bool NavPoseTracker::get_nearest_task_point(PlanarPose * pose, double * distance) const
{
  if (pose == nullptr || distance == nullptr) {
    return false;
  }

  std::vector<PlanarPose> task_points;
  {
    std::lock_guard<std::mutex> lock(task_points_mutex_);
    task_points = task_points_;
  }
  if (task_points.empty()) {
    return false;
  }

  PlanarPose arm_pose;
  if (!get_arm_pose(&arm_pose)) {
    return false;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  const PlanarPose * best_point = nullptr;
  for (const auto & point : task_points) {
    const double dx = point.x - arm_pose.x;
    const double dy = point.y - arm_pose.y;
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

  PlanarPose task_pose;
  if (!get_task_point_pose(task_id, &task_pose)) {
    return false;
  }

  return compute_relative_pose_to_task_pose(task_pose, pose);
}

bool NavPoseTracker::compute_relative_pose_to_task_pose(
  const PlanarPose & task_pose,
  RelativePlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  PlanarPose lidar_pose;
  PlanarPose arm_pose;
  if (!get_lidar_pose(&lidar_pose) || !get_arm_pose(&arm_pose)) {
    return false;
  }

  *pose = build_relative_pose(lidar_pose, arm_pose, task_pose);
  return true;
}

bool NavPoseTracker::compute_relative_pose_to_nearest(RelativePlanarPose * pose) const
{
  if (pose == nullptr) {
    return false;
  }

  PlanarPose task_pose;
  double distance = 0.0;
  if (!get_nearest_task_point(&task_pose, &distance)) {
    return false;
  }

  if (!compute_relative_pose_to_task_pose(task_pose, pose)) {
    return false;
  }
  pose->distance = distance;
  return true;
}

double NavPoseTracker::yaw_from_quaternion_wxyz(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

PlanarPose NavPoseTracker::arm_pose_from_lidar_pose(
  const PlanarPose & lidar_pose,
  const PlanarTransform & lidar_in_arm)
{
  if (!lidar_in_arm.enabled) {
    return lidar_pose;
  }

  // map_T_lidar = map_T_arm * arm_T_lidar, so map_T_arm = map_T_lidar * inverse(arm_T_lidar).
  PlanarPose arm_pose = lidar_pose;
  arm_pose.yaw = normalize_angle(lidar_pose.yaw - lidar_in_arm.yaw);

  const double cos_yaw = std::cos(arm_pose.yaw);
  const double sin_yaw = std::sin(arm_pose.yaw);
  const double offset_x_world = cos_yaw * lidar_in_arm.x - sin_yaw * lidar_in_arm.y;
  const double offset_y_world = sin_yaw * lidar_in_arm.x + cos_yaw * lidar_in_arm.y;
  arm_pose.x = lidar_pose.x - offset_x_world;
  arm_pose.y = lidar_pose.y - offset_y_world;
  return arm_pose;
}

PlanarTransform NavPoseTracker::resolve_lidar_in_arm_transform()
{
  auto resolved = lidar_in_arm_;
  if (!resolved.enabled || !resolved.prefer_tf || !tf_buffer_) {
    return resolved;
  }

  try {
    const auto tf = tf_buffer_->lookupTransform(
      resolved.parent_frame,
      resolved.child_frame,
      tf2::TimePointZero);

    resolved.x = tf.transform.translation.x;
    resolved.y = tf.transform.translation.y;
    resolved.yaw = yaw_from_quaternion_wxyz(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);

    if (!logged_tf_lookup_success_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[nav_pose] Using TF %s -> %s: x=%.3f y=%.3f yaw=%.3f rad.",
        resolved.parent_frame.c_str(), resolved.child_frame.c_str(),
        resolved.x, resolved.y, resolved.yaw);
      logged_tf_lookup_success_ = true;
    }
  } catch (const tf2::TransformException & ex) {
    if (!warned_tf_lookup_failure_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav_pose] TF lookup %s -> %s failed: %s. Falling back to configured lidar_extrinsics.",
        resolved.parent_frame.c_str(), resolved.child_frame.c_str(), ex.what());
      warned_tf_lookup_failure_ = true;
    }
  }

  return resolved;
}

}  // namespace arm2_task::task
