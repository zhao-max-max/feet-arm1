#include "arm2_task/task/motion_client.hpp"

#include <future>
#include <sstream>
#include <utility>

#include "arm2_task/task/timing_events.hpp"

using namespace std::chrono_literals;

namespace arm2_task::task
{

MotionClient::MotionClient(
  rclcpp::Node * node,
  rclcpp_action::Client<MoveJoint>::SharedPtr move_joint_client,
  rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client,
  const std::atomic<bool> * is_running)
: node_(node),
  move_joint_client_(std::move(move_joint_client)),
  mode_client_(std::move(mode_client)),
  is_running_(is_running)
{
  timing_event_pub_ = create_timing_event_publisher(node_);
}

void MotionClient::set_trajectory_defaults(
  double max_velocity,
  double max_acceleration,
  double blend_radius)
{
  max_v_ = max_velocity;
  max_a_ = max_acceleration;
  blend_radius_ = blend_radius;
}

bool MotionClient::task_is_running() const
{
  return is_running_ == nullptr || is_running_->load();
}

bool MotionClient::send_move_goal(const std::vector<Eigen::VectorXd> & q_waypoints)
{
  active_move_goal_sequence_ = ++move_goal_sequence_;
  std::ostringstream detail;
  detail << "seq=" << active_move_goal_sequence_
         << ",points=" << q_waypoints.size()
         << ",max_v=" << max_v_
         << ",max_a=" << max_a_
         << ",blend_radius=" << blend_radius_;
  publish_timing_event(node_, timing_event_pub_, "motion", "move_joint", "begin", detail.str());

  if (!move_joint_client_->wait_for_action_server(10s)) {
    is_action_running_ = false;
    action_finished_ = true;
    {
      std::lock_guard<std::mutex> lock(action_result_mutex_);
      last_action_succeeded_ = false;
      last_action_message_ = "move_joint action server not available.";
    }
    RCLCPP_ERROR(node_->get_logger(), "move_joint action server not available.");
    publish_timing_event(
      node_, timing_event_pub_, "motion", "move_joint", "end",
      detail.str() + ",ok=0,reason=action_server_unavailable");
    return false;
  }

  MoveJoint::Goal goal_msg;
  goal_msg.max_velocity = max_v_;
  goal_msg.max_acceleration = max_a_;
  goal_msg.blend_radius = blend_radius_;
  goal_msg.num_points = static_cast<int32_t>(q_waypoints.size());
  for (const auto & q : q_waypoints) {
    for (int i = 0; i < 5; ++i) {
      goal_msg.joint_targets.push_back(q[i]);
    }
  }

  auto send_goal_options = rclcpp_action::Client<MoveJoint>::SendGoalOptions();
  action_finished_ = false;
  is_action_running_ = true;
  {
    std::lock_guard<std::mutex> lock(action_result_mutex_);
    last_action_succeeded_ = false;
    last_action_message_.clear();
  }

  send_goal_options.goal_response_callback =
    [this](std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
      if (!goal_handle) {
        is_action_running_ = false;
        action_finished_ = true;
        {
          std::lock_guard<std::mutex> lock(action_result_mutex_);
          last_action_succeeded_ = false;
          last_action_message_ = "move_joint goal rejected by action server.";
        }
        RCLCPP_ERROR(node_->get_logger(), "move_joint goal was rejected by the action server.");
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleMoveJoint::WrappedResult & result) {
      is_action_running_ = false;
      action_finished_ = true;
      bool succeeded = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      std::string msg;
      if (result.result) {
        succeeded = succeeded && result.result->success;
        msg = result.result->message;
      } else if (!succeeded) {
        msg = "Action finished without a result payload.";
      }
      if (msg.empty()) {
        msg = succeeded ? "success" : "move_joint action failed.";
      }
      {
        std::lock_guard<std::mutex> lock(action_result_mutex_);
        last_action_succeeded_ = succeeded;
        last_action_message_ = msg;
      }
      if (!succeeded) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "move_joint action failed. code=%d message=%s",
          static_cast<int>(result.code), msg.c_str());
      }
    };

  move_joint_client_->async_send_goal(goal_msg, send_goal_options);
  return true;
}

bool MotionClient::send_move_goal(const Eigen::VectorXd & q_single)
{
  return send_move_goal(std::vector<Eigen::VectorXd>{q_single});
}

bool MotionClient::wait_for_action_completion(std::chrono::seconds timeout)
{
  const int seq = active_move_goal_sequence_;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && task_is_running() && !action_finished_) {
    if (std::chrono::steady_clock::now() >= deadline) {
      is_action_running_ = false;
      {
        std::lock_guard<std::mutex> lock(action_result_mutex_);
        last_action_succeeded_ = false;
        last_action_message_ = "Timed out waiting for move_joint action.";
      }
      RCLCPP_ERROR(
        node_->get_logger(),
        "Timed out waiting for move_joint action after %lds.", timeout.count());
      publish_timing_event(
        node_, timing_event_pub_, "motion", "move_joint", "end",
        "seq=" + std::to_string(seq) + ",ok=0,reason=timeout");
      return false;
    }
    rclcpp::sleep_for(50ms);
  }

  if ((!rclcpp::ok() || !task_is_running()) && !action_finished_) {
    std::lock_guard<std::mutex> lock(action_result_mutex_);
    last_action_succeeded_ = false;
    last_action_message_ = "Task loop stopped before action finished.";
    publish_timing_event(
      node_, timing_event_pub_, "motion", "move_joint", "end",
      "seq=" + std::to_string(seq) + ",ok=0,reason=task_stopped");
    return false;
  }

  bool succeeded;
  std::string msg;
  {
    std::lock_guard<std::mutex> lock(action_result_mutex_);
    succeeded = last_action_succeeded_;
    msg = last_action_message_;
  }
  action_finished_ = false;
  if (!succeeded) {
    RCLCPP_ERROR(
      node_->get_logger(), "move_joint action reported failure: %s",
      msg.empty() ? "unknown error" : msg.c_str());
  }
  publish_timing_event(
    node_, timing_event_pub_, "motion", "move_joint", "end",
    "seq=" + std::to_string(seq) + ",ok=" + timing_bool(rclcpp::ok() && succeeded) +
    ",message=" + (msg.empty() ? "none" : msg));
  return rclcpp::ok() && succeeded;
}

int MotionClient::request_mode_switch(const std::string & mode_name)
{
  const std::string detail = "mode=" + mode_name;
  publish_timing_event(node_, timing_event_pub_, "service", "set_controller_mode", "begin", detail);
  if (!mode_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "Mode switch service not available");
    publish_timing_event(
      node_, timing_event_pub_, "service", "set_controller_mode", "end",
      detail + ",ok=0,reason=service_unavailable");
    return 0;
  }

  auto request = std::make_shared<robot_msgs::srv::SetControllerMode::Request>();
  request->mode = mode_name;

  auto result_future = mode_client_->async_send_request(request);
  if (result_future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Mode switch to [%s] timed out.", mode_name.c_str());
    publish_timing_event(
      node_, timing_event_pub_, "service", "set_controller_mode", "end",
      detail + ",ok=0,reason=timeout");
    return 0;
  }

  const auto response = result_future.get();
  if (!response || !response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Mode switch to [%s] failed.", mode_name.c_str());
    publish_timing_event(
      node_, timing_event_pub_, "service", "set_controller_mode", "end",
      detail + ",ok=0,reason=response_failure");
    return 0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "\033[1;36m[Mode]\033[0m Controller switched to: %s", mode_name.c_str());
  publish_timing_event(
    node_, timing_event_pub_, "service", "set_controller_mode", "end", detail + ",ok=1");
  return 1;
}

}  // namespace arm2_task::task
