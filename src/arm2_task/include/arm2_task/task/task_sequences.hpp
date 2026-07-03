#pragma once

#include <map>
#include <string>

#include <Eigen/Dense>

#include "arm2_task/task/motion_client.hpp"
#include "arm2_task/task/perception_client.hpp"
#include "arm2_task/task/task_primitives.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm2_task::task
{

class TaskSequences
{
public:
  struct Config
  {
    std::string pick_object_name{"box"};
    std::string place_frame_name{"target_frame"};
    std::string stack_service_name{"get_stack_pos"};

    bool use_mock_grasp_target{false};
    double grasp_mock_x{0.35};
    double grasp_mock_y{0.0};
    double grasp_mock_z{0.12};

    bool use_mock_place_frame{false};
    double place_mock_x{0.35};
    double place_mock_y{0.0};
    double place_mock_z{0.0};
    double place_mock_yaw{0.0};

    bool use_mock_stack{false};
    double stack_mock_x{0.35};
    double stack_mock_y{0.0};
    double stack_mock_z{0.1};
    double stack_mock_yaw{0.0};
  };

  TaskSequences(
    rclcpp::Node * node,
    MotionClient * motion_client,
    PerceptionClient * perception_client,
    TaskPrimitives * primitives,
    const std::map<std::string, Eigen::VectorXd> * presets,
    Config config);

  bool grasp_from_perception();
  bool grasp_mock_or_perception();
  bool grasp_pose(const geometry_msgs::msg::Pose & target, bool aligned);

  bool place_from_perception();
  bool place_mock_or_perception();
  bool place_pose(const geometry_msgs::msg::Pose & frame_pose);

  bool stack_mock_or_perception();
  bool stack_pose(const geometry_msgs::msg::Pose & box_top_pose);

  bool move_to_carry_loaded();

private:
  static geometry_msgs::msg::Pose make_forward_pose();
  static geometry_msgs::msg::Pose make_yaw_pose(
    double x,
    double y,
    double z,
    double yaw);

  rclcpp::Node * node_{nullptr};
  MotionClient * motion_client_{nullptr};
  PerceptionClient * perception_client_{nullptr};
  TaskPrimitives * primitives_{nullptr};
  const std::map<std::string, Eigen::VectorXd> * presets_{nullptr};
  Config config_;
};

}  // namespace arm2_task::task
