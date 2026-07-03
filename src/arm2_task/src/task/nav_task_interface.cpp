#include "arm2_task/task/nav_task_interface.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace arm2_task::task
{

NavTaskInterface::NavTaskInterface(
  rclcpp::Node * node,
  TaskSequences * sequences,
  TaskPrimitives * primitives,
  NavPoseTracker * nav_pose_tracker,
  const std::atomic<bool> * is_running)
: node_(node),
  sequences_(sequences),
  primitives_(primitives),
  nav_pose_tracker_(nav_pose_tracker),
  is_running_(is_running)
{
  arm_mission_server_ = node_->create_service<std_srvs::srv::Trigger>(
    "/arm/mission_event",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      response->success = true;
      response->message = "received";
      RCLCPP_INFO(node_->get_logger(), "[nav] Mission trigger received from navigation.");
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      pending_trigger_ = true;
      cmd_cv_.notify_one();
    });

  nav_event_client_ = node_->create_client<navigation::srv::StringCommand>(
    "/navigation/arm_event");

  RCLCPP_INFO(
    node_->get_logger(),
    "Nav integration ready: /arm/mission_event server + /navigation/arm_event client.");
}

void NavTaskInterface::log_nav_pose_snapshot(const char * context)
{
  if (nav_pose_tracker_ == nullptr) {
    return;
  }

  arm2_task::task::PlanarPose robot_pose;
  if (!nav_pose_tracker_->get_robot_pose(&robot_pose)) {
    RCLCPP_WARN(node_->get_logger(), "[nav] %s: no /navigation/state received yet.", context);
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] %s robot pose: x=%.3f y=%.3f yaw=%.3f rad",
    context, robot_pose.x, robot_pose.y, robot_pose.yaw);

  arm2_task::task::RelativePlanarPose relative_pose;
  if (!nav_pose_tracker_->compute_relative_pose_to_nearest(&relative_pose)) {
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] %s nearest task point %d: world=(%.3f, %.3f, %.3f) body=(%.3f, %.3f, %.3f) dist=%.3f",
    context,
    relative_pose.task_pose.id,
    relative_pose.task_pose.x,
    relative_pose.task_pose.y,
    relative_pose.task_pose.yaw,
    relative_pose.dx_body,
    relative_pose.dy_body,
    relative_pose.dyaw,
    relative_pose.distance);
}

void NavTaskInterface::send_nav_event(const std::string & event)
{
  if (!nav_event_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] /navigation/arm_event service unavailable, dropping event: %s",
      event.c_str());
    return;
  }

  auto req = std::make_shared<navigation::srv::StringCommand::Request>();
  req->message = event;
  auto future = nav_event_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(), "[nav] arm_event \"%s\" timed out.", event.c_str());
    return;
  }

  auto resp = future.get();
  if (resp->success) {
    RCLCPP_INFO(node_->get_logger(), "[nav] arm_event \"%s\" acknowledged.", event.c_str());
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "[nav] arm_event \"%s\" rejected: %s",
      event.c_str(), resp->message.c_str());
  }
}

bool NavTaskInterface::do_grasp_sequence()
{
  if (!sequences_->grasp_from_perception()) {
    return false;
  }

  rclcpp::sleep_for(500ms);
  send_nav_event("grabbed");

  if (!sequences_->move_to_carry_loaded()) {
    return false;
  }

  send_nav_event("completed");
  return true;
}

bool NavTaskInterface::do_place_sequence()
{
  if (!sequences_->place_from_perception()) {
    return false;
  }

  // TODO: 当 nav 支持放置任务点时，在这里发送适当的事件。
  primitives_->do_reset();
  return true;
}

void NavTaskInterface::run()
{
  RCLCPP_INFO(node_->get_logger(), "[nav] Ready. Waiting for /arm/mission_event triggers...");

  while (rclcpp::ok() && is_running_->load()) {
    {
      std::unique_lock<std::mutex> lk(cmd_mutex_);
      cmd_cv_.wait_for(
        lk,
        std::chrono::milliseconds(200),
        [this] { return pending_trigger_; });
      if (!pending_trigger_) {
        continue;
      }
      pending_trigger_ = false;
    }

    if (remote_busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "[nav] Trigger received but arm is busy, ignoring.");
      continue;
    }

    const auto current_state = state_;
    if (current_state == arm2_task::TaskState::IDLE) {
      log_nav_pose_snapshot("pickup trigger");
      RCLCPP_INFO(node_->get_logger(), "[nav] Executing grasp sequence...");
      if (do_grasp_sequence()) {
        state_ = arm2_task::TaskState::HOLDING;
      }
    } else if (current_state == arm2_task::TaskState::HOLDING) {
      log_nav_pose_snapshot("place trigger");
      // TODO: 当 nav 支持放置任务点时，启用 do_place_sequence 并回到 IDLE。
      RCLCPP_WARN(node_->get_logger(), "[nav] Place sequence not yet wired to nav. Ignoring trigger.");
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "[nav] Unexpected state %d on trigger, ignoring.",
        static_cast<int>(current_state));
    }

    remote_busy_.store(false);
  }
}

}  // namespace arm2_task::task
