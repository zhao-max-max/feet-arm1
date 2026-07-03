#include "arm2_task/task/pose_utils.hpp"

#include <cmath>

namespace arm2_task::task
{

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double object_yaw_roll(const geometry_msgs::msg::Pose & object_world)
{
  const double object_yaw = std::atan2(object_world.position.y, object_world.position.x);

  // do_look_out and solveIK both align joint_0 to atan2(y, x), so the default tool roll is 0.
  const double base_yaw = object_yaw;
  return normalize_angle(object_yaw - base_yaw);
}

EdgeAlignedRoll compute_edge_aligned_roll(const geometry_msgs::msg::Pose & world_pose)
{
  EdgeAlignedRoll result;
  const auto & q = world_pose.orientation;

  if (
    std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 &&
    std::abs(q.z) < 1e-6 && std::abs(q.w - 1.0) < 1e-6)
  {
    result.identity_orientation = true;
    return result;
  }

  const double ux = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double uy = 2.0 * (q.x * q.y + q.z * q.w);
  result.alpha = std::atan2(uy, ux);
  result.chosen = result.alpha;

  for (int k = 0; k < 4; ++k) {
    const double candidate = normalize_angle(result.alpha + k * M_PI / 2.0);
    if (candidate >= -M_PI - 1e-9 && candidate <= -M_PI / 2.0 + 1e-9) {
      result.chosen = candidate;
      break;
    }
  }

  result.edge_yaw = result.chosen + M_PI;
  result.base_yaw = std::atan2(world_pose.position.y, world_pose.position.x) + M_PI / 2.0;
  result.roll = normalize_angle(result.base_yaw - result.edge_yaw);
  return result;
}

double compute_stack_tool_roll(
  const geometry_msgs::msg::Pose & box_top_world,
  double roll_sign)
{
  const auto & q = box_top_world.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double base_yaw = std::atan2(box_top_world.position.y, box_top_world.position.x);
  return normalize_angle(roll_sign * (yaw - base_yaw));
}

}  // namespace arm2_task::task
