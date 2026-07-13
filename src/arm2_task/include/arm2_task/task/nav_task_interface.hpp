#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "arm2_task/common_units.hpp"
#include "arm2_task/task/nav_pose_tracker.hpp"
#include "arm2_task/task/task_primitives.hpp"
#include "arm2_task/task/task_sequences.hpp"
#include "navigation/srv/mission_command.hpp"
#include "navigation/srv/string_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace arm2_task::task
{

class NavTaskInterface
{
public:
  struct Config
  {
    double place_height{0.28};
    bool radar_pick_fallback_enabled{true};
    double radar_pick_fallback_target_z{0.12};
    double stack_fallback_target_z{0.25};
    int stack_on_place_index{1};  // 第几次 place 用 stack（0-based），默认第2次
  };

  NavTaskInterface(
    rclcpp::Node * node,
    TaskSequences * sequences,
    TaskPrimitives * primitives,
    NavPoseTracker * nav_pose_tracker,
    const std::atomic<bool> * is_running,
    Config config);

  void run();

private:
  struct MissionCommand
  {
    std::uint32_t task_index{0};
    int point_id{0};
    std::string action;
    double x{0.0};
    double y{0.0};
  };

  void log_nav_pose_snapshot(const char * context);
  void log_mission_command(const MissionCommand & command);
  void send_nav_event(const std::string & event);
  void publish_mission_request_debug(const navigation::srv::MissionCommand::Request & request);
  void publish_mission_response_debug(const navigation::srv::MissionCommand::Response & response);
  void publish_nav_event_request_debug(const navigation::srv::StringCommand::Request & request);
  void publish_nav_event_response_debug(const navigation::srv::StringCommand::Response & response);
  const char * task_state_name(arm2_task::TaskState state) const;
  void set_task_state(arm2_task::TaskState next_state, const std::string & reason);
  bool compute_command_relative_pose(
    const MissionCommand & command,
    RelativePlanarPose * relative_pose);
  bool do_lookout_align_for_command(
    const MissionCommand & command,
    const char * context);
  bool do_ready_sequence(const MissionCommand & command);
  bool do_grasp_sequence(const MissionCommand & command);
  bool do_radar_pick_fallback(const MissionCommand & command);
  bool do_place_sequence(const MissionCommand & command);
  bool do_store_sequence(const MissionCommand & command);
  bool do_pickup_from_dog_sequence(const MissionCommand & command);
  bool execute_mission_command(const MissionCommand & command);
  void cache_active_ready_command(const MissionCommand & command);
  void clear_active_ready_command();
  std::optional<MissionCommand> active_ready_command_copy();
  bool do_pickup_lookout_align(const MissionCommand & command);
  bool handle_debug_state_command(
    const navigation::srv::MissionCommand::Request & request,
    navigation::srv::MissionCommand::Response * response);

  rclcpp::Node * node_{nullptr};
  TaskSequences * sequences_{nullptr};
  TaskPrimitives * primitives_{nullptr};
  NavPoseTracker * nav_pose_tracker_{nullptr};
  const std::atomic<bool> * is_running_{nullptr};
  Config config_;

  arm2_task::TaskState state_{arm2_task::TaskState::IDLE};
  std::atomic<bool> remote_busy_{false};
  int completed_place_count_{0};  // 记录已完成的 place 次数，第2次(count==1)走 stack
  rclcpp::Service<navigation::srv::MissionCommand>::SharedPtr arm_mission_server_;
  rclcpp::Service<navigation::srv::MissionCommand>::SharedPtr debug_state_server_;
  rclcpp::Client<navigation::srv::StringCommand>::SharedPtr nav_event_client_;
  rclcpp::Publisher<navigation::srv::MissionCommand::Request>::SharedPtr mission_request_debug_pub_;
  rclcpp::Publisher<navigation::srv::MissionCommand::Response>::SharedPtr mission_response_debug_pub_;
  rclcpp::Publisher<navigation::srv::StringCommand::Request>::SharedPtr nav_event_request_debug_pub_;
  rclcpp::Publisher<navigation::srv::StringCommand::Response>::SharedPtr nav_event_response_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_event_pub_;
  std::mutex cmd_mutex_;
  std::condition_variable cmd_cv_;
  std::optional<MissionCommand> pending_command_;
  std::optional<MissionCommand> active_ready_command_;
};

}  // namespace arm2_task::task
