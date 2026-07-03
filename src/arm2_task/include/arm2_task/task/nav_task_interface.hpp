#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>

#include "arm2_task/common_units.hpp"
#include "arm2_task/task/nav_pose_tracker.hpp"
#include "arm2_task/task/task_primitives.hpp"
#include "arm2_task/task/task_sequences.hpp"
#include "navigation/srv/string_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace arm2_task::task
{

class NavTaskInterface
{
public:
  NavTaskInterface(
    rclcpp::Node * node,
    TaskSequences * sequences,
    TaskPrimitives * primitives,
    NavPoseTracker * nav_pose_tracker,
    const std::atomic<bool> * is_running);

  void run();

private:
  void log_nav_pose_snapshot(const char * context);
  void send_nav_event(const std::string & event);
  bool do_grasp_sequence();
  bool do_place_sequence();

  rclcpp::Node * node_{nullptr};
  TaskSequences * sequences_{nullptr};
  TaskPrimitives * primitives_{nullptr};
  NavPoseTracker * nav_pose_tracker_{nullptr};
  const std::atomic<bool> * is_running_{nullptr};

  arm2_task::TaskState state_{arm2_task::TaskState::IDLE};
  std::atomic<bool> remote_busy_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_mission_server_;
  rclcpp::Client<navigation::srv::StringCommand>::SharedPtr nav_event_client_;
  std::mutex cmd_mutex_;
  std::condition_variable cmd_cv_;
  bool pending_trigger_{false};
};

}  // namespace arm2_task::task
