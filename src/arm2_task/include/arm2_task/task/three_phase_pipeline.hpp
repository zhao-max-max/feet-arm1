#pragma once

#include <map>
#include <mutex>
#include <string>

#include <Eigen/Dense>

#include "arm2_task/kinematics_engine.hpp"
#include "arm2_task/task/motion_client.hpp"
#include "arm2_task/task/perception_client.hpp"
#include "arm2_task/task/task_primitives.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm2_task::task
{

class ThreePhasePipeline
{
public:
  struct Config
  {
    std::string pick_object_name{"box"};
    double align_threshold{0.005};
    int align_max_iters{5};
  };

  ThreePhasePipeline(
    rclcpp::Node * node,
    arm2_task::KinematicsEngine * kin_engine,
    MotionClient * motion_client,
    PerceptionClient * perception_client,
    TaskPrimitives * primitives,
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub,
    const std::map<std::string, Eigen::VectorXd> * presets,
    Eigen::VectorXd * q_current,
    std::mutex * state_mutex,
    Config config);

  bool do_grasp();
  bool do_place();

private:
  static geometry_msgs::msg::Pose make_forward_pose();

  bool phase1_get_coarse_target(geometry_msgs::msg::Pose & target_world);
  bool phase2_align(geometry_msgs::msg::Pose & target_world);
  bool phase3_grasp_descend(const geometry_msgs::msg::Pose & target_world);
  bool phase3_place_descend(const geometry_msgs::msg::Pose & target_world);
  Eigen::VectorXd current_q_snapshot() const;

  rclcpp::Node * node_{nullptr};
  arm2_task::KinematicsEngine * kin_engine_{nullptr};
  MotionClient * motion_client_{nullptr};
  PerceptionClient * perception_client_{nullptr};
  TaskPrimitives * primitives_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
  const std::map<std::string, Eigen::VectorXd> * presets_{nullptr};
  Eigen::VectorXd * q_current_{nullptr};
  std::mutex * state_mutex_{nullptr};
  Config config_;
};

}  // namespace arm2_task::task
