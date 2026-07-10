#pragma once

#include <atomic>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "arm2_task/kinematics_engine.hpp"
#include "arm2_task/task/end_effector_client.hpp"
#include "arm2_task/task/motion_client.hpp"
#include "arm2_task/task/perception_client.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace arm2_task::task
{

class TaskPrimitives
{
public:
  struct Config
  {
    bool require_suction_service{false};
    std::string pick_object_name{"box"};
    double grasp_pitch{-1.57};
    double tool_pitch_offset{0.0};
    double tool_yaw_offset{0.0};
    double object_height{0.05};
    double pre_grasp_offset{0.10};
    double pre_place_offset{0.12};
    double place_retreat_offset{0.15};
    double tool_offset_x{0.0};
    double tool_offset_y{0.0};
    double tool_offset_z{0.0};
    double tool_tip_length{0.0};
    double place_frame_hover_height{0.25};
    double place_frame_contact_offset{0.0};
    double stack_hover_height{0.05};
    double stack_contact_offset{0.25};
    double stack_roll_sign{1.0};
    double store_hover_offset{0.10};  // 狗背交接时悬停高度偏移 (m)
    double store_roll_offset{0.0};    // 放到狗吸盘时 joint_4 附加转角 (rad)
    double grasp_roll_offset{0.0};    // 抓取时 joint_4 roll 附加偏置 (rad)
  };

  TaskPrimitives(
    rclcpp::Node * node,
    arm2_task::KinematicsEngine * kin_engine,
    MotionClient * motion_client,
    PerceptionClient * perception_client,
    EndEffectorClient * end_effector_client,
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub,
    const std::map<std::string, Eigen::VectorXd> * presets,
    Eigen::VectorXd * q_current,
    Eigen::VectorXd * dq_current,
    std::mutex * state_mutex,
    const std::atomic<bool> * is_running,
    Config config);

  double get_object_yaw(const geometry_msgs::msg::Pose & object_world) const;
  double get_box_edge_roll(const geometry_msgs::msg::Pose & world_pose);
  double get_frame_yaw(const geometry_msgs::msg::Pose & frame_world) const;

  void wait_joints_still(double dq_threshold = 0.06, int timeout_ms = 1000);

  void do_reset();
  void do_reset_suction();
  void do_load();
  void do_store_pose();
  void do_dog_suction_on();
  void do_dog_suction_off();
  bool do_look_out(const geometry_msgs::msg::Pose & target);
  void do_suction_on();
  void do_suction_off();

  bool do_grasp_move(const geometry_msgs::msg::Pose & target, double tool_roll);
  bool do_grasp_move(const geometry_msgs::msg::Pose & target);
  bool do_place_move(const geometry_msgs::msg::Pose & target);
  bool do_place_move_with_orientation(const geometry_msgs::msg::Pose & frame_world);
  bool do_place_move_with_direct_height(const geometry_msgs::msg::Pose & target);
  bool do_stack_move_with_orientation(const geometry_msgs::msg::Pose & box_top_world);

  bool do_full_grasp(const geometry_msgs::msg::Pose & target);
  bool do_full_grasp_aligned(const geometry_msgs::msg::Pose & target);
  bool do_grasp_from_current_view();
  bool do_full_place(const geometry_msgs::msg::Pose & target);

private:
  bool task_is_running() const;
  bool send_move_goal(const std::vector<Eigen::VectorXd> & q_waypoints);
  bool send_move_goal(const Eigen::VectorXd & q_single);
  bool wait_for_action_completion(std::chrono::seconds timeout = std::chrono::seconds(30));
  int request_mode_switch(const std::string & mode_name);
  int set_suction(bool activate);
  double apply_roll_continuity(double roll);
  Eigen::VectorXd current_q_snapshot() const;
  Eigen::VectorXd current_dq_snapshot() const;

  rclcpp::Node * node_{nullptr};
  arm2_task::KinematicsEngine * kin_engine_{nullptr};
  MotionClient * motion_client_{nullptr};
  PerceptionClient * perception_client_{nullptr};
  EndEffectorClient * end_effector_client_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
  const std::map<std::string, Eigen::VectorXd> * presets_{nullptr};
  Eigen::VectorXd * q_current_{nullptr};
  Eigen::VectorXd * dq_current_{nullptr};
  std::mutex * state_mutex_{nullptr};
  const std::atomic<bool> * is_running_{nullptr};
  Config config_;
  double last_grasp_roll_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_event_pub_;
};

}  // namespace arm2_task::task
