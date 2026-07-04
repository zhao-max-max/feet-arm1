#include "arm2_task/task/nav_task_interface.hpp"

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/pose.hpp"

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
  const auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  mission_request_debug_pub_ =
    node_->create_publisher<navigation::srv::MissionCommand::Request>(
    "/debug/nav/mission_request", debug_qos);
  mission_response_debug_pub_ =
    node_->create_publisher<navigation::srv::MissionCommand::Response>(
    "/debug/nav/mission_response", debug_qos);
  nav_event_request_debug_pub_ =
    node_->create_publisher<navigation::srv::StringCommand::Request>(
    "/debug/nav/arm_event_request", debug_qos);
  nav_event_response_debug_pub_ =
    node_->create_publisher<navigation::srv::StringCommand::Response>(
    "/debug/nav/arm_event_response", debug_qos);

  arm_mission_server_ = node_->create_service<navigation::srv::MissionCommand>(
    "/arm/mission_event",
    [this](
      const navigation::srv::MissionCommand::Request::SharedPtr request,
      navigation::srv::MissionCommand::Response::SharedPtr response)
    {
      if (request) {
        publish_mission_request_debug(*request);
      } else {
        publish_mission_request_debug(navigation::srv::MissionCommand::Request{});
      }

      if (!request ||
          (request->action != "ready" && request->action != "pickup" && request->action != "place")) {
        response->success = false;
        response->message = "invalid mission action";
        publish_mission_response_debug(*response);
        RCLCPP_WARN(
          node_->get_logger(), "[nav] Rejected mission command: invalid action='%s'.",
          request ? request->action.c_str() : "<null>");
        return;
      }

      if (remote_busy_.load()) {
        response->success = false;
        response->message = "arm busy";
        publish_mission_response_debug(*response);
        RCLCPP_WARN(
          node_->get_logger(),
          "[nav] Rejected mission %s task_index=%u point_id=%d: arm busy.",
          request->action.c_str(), request->task_index, request->point_id);
        return;
      }

      const auto current_state = state_;
      if (request->action == "ready" &&
          current_state != arm2_task::TaskState::IDLE &&
          current_state != arm2_task::TaskState::LOOKOUT) {
        response->success = false;
        response->message = "arm is not idle";
        publish_mission_response_debug(*response);
        RCLCPP_WARN(
          node_->get_logger(),
          "[nav] Rejected ready task_index=%u point_id=%d: arm state=%d, expected IDLE/LOOKOUT.",
          request->task_index, request->point_id, static_cast<int>(current_state));
        return;
      }
      if (request->action == "pickup" &&
          current_state != arm2_task::TaskState::IDLE &&
          current_state != arm2_task::TaskState::LOOKOUT) {
        response->success = false;
        response->message = "arm is not ready for pickup";
        publish_mission_response_debug(*response);
        RCLCPP_WARN(
          node_->get_logger(),
          "[nav] Rejected pickup task_index=%u point_id=%d: arm state=%d, expected IDLE/LOOKOUT.",
          request->task_index, request->point_id, static_cast<int>(current_state));
        return;
      }
      if (request->action == "place" && current_state != arm2_task::TaskState::HOLDING) {
        response->success = false;
        response->message = "arm is not holding";
        publish_mission_response_debug(*response);
        RCLCPP_WARN(
          node_->get_logger(),
          "[nav] Rejected place task_index=%u point_id=%d: arm state=%d, expected HOLDING.",
          request->task_index, request->point_id, static_cast<int>(current_state));
        return;
      }

      MissionCommand command;
      command.task_index = request->task_index;
      command.point_id = request->point_id;
      command.action = request->action;
      command.x = request->x;
      command.y = request->y;

      {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        if (pending_command_.has_value()) {
          response->success = false;
          response->message = "mission command pending";
          publish_mission_response_debug(*response);
          RCLCPP_WARN(
            node_->get_logger(),
            "[nav] Rejected mission %s task_index=%u point_id=%d: previous command pending.",
            request->action.c_str(), request->task_index, request->point_id);
          return;
        }
        pending_command_ = command;
      }

      response->success = true;
      response->message = "received";
      publish_mission_response_debug(*response);
      RCLCPP_INFO(
        node_->get_logger(),
        "[nav] Mission command received: action=%s task_index=%u point_id=%d map=(%.3f, %.3f).",
        command.action.c_str(), command.task_index, command.point_id, command.x, command.y);
      cmd_cv_.notify_one();
    });

  nav_event_client_ = node_->create_client<navigation::srv::StringCommand>(
    "/navigation/arm_event");

  RCLCPP_INFO(
    node_->get_logger(),
    "Nav integration ready: /arm/mission_event MissionCommand server + /navigation/arm_event client.");
}

void NavTaskInterface::publish_mission_request_debug(
  const navigation::srv::MissionCommand::Request & request)
{
  if (mission_request_debug_pub_) {
    mission_request_debug_pub_->publish(request);
  }
}

void NavTaskInterface::publish_mission_response_debug(
  const navigation::srv::MissionCommand::Response & response)
{
  if (mission_response_debug_pub_) {
    mission_response_debug_pub_->publish(response);
  }
}

void NavTaskInterface::publish_nav_event_request_debug(
  const navigation::srv::StringCommand::Request & request)
{
  if (nav_event_request_debug_pub_) {
    nav_event_request_debug_pub_->publish(request);
  }
}

void NavTaskInterface::publish_nav_event_response_debug(
  const navigation::srv::StringCommand::Response & response)
{
  if (nav_event_response_debug_pub_) {
    nav_event_response_debug_pub_->publish(response);
  }
}

void NavTaskInterface::log_nav_pose_snapshot(const char * context)
{
  if (nav_pose_tracker_ == nullptr) {
    return;
  }

  arm2_task::task::PlanarPose robot_pose;
  arm2_task::task::PlanarPose lidar_pose;
  if (!nav_pose_tracker_->get_arm_pose(&robot_pose) ||
      !nav_pose_tracker_->get_lidar_pose(&lidar_pose)) {
    RCLCPP_WARN(node_->get_logger(), "[nav] %s: no /navigation/state received yet.", context);
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] %s lidar pose(map): x=%.3f y=%.3f yaw=%.3f rad",
    context, lidar_pose.x, lidar_pose.y, lidar_pose.yaw);

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] %s arm pose(map): x=%.3f y=%.3f yaw=%.3f rad",
    context, robot_pose.x, robot_pose.y, robot_pose.yaw);

  arm2_task::task::RelativePlanarPose relative_pose;
  if (!nav_pose_tracker_->compute_relative_pose_to_nearest(&relative_pose)) {
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] %s nearest task point %d: map=(%.3f, %.3f, %.3f) arm_to_task=(x=%.3f, y=%.3f, yaw=%.3f) dist=%.3f",
    context,
    relative_pose.task_pose.id,
    relative_pose.task_pose.x,
    relative_pose.task_pose.y,
    relative_pose.task_pose.yaw,
    relative_pose.x,
    relative_pose.y,
    relative_pose.yaw,
    relative_pose.distance);
}

void NavTaskInterface::log_mission_command(const MissionCommand & command)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] mission command: action=%s task_index=%u point_id=%d map=(%.3f, %.3f)",
    command.action.c_str(), command.task_index, command.point_id, command.x, command.y);

  if (nav_pose_tracker_ == nullptr) {
    return;
  }

  arm2_task::task::PlanarPose lidar_pose;
  arm2_task::task::PlanarPose arm_pose;
  if (!nav_pose_tracker_->get_lidar_pose(&lidar_pose) ||
      !nav_pose_tracker_->get_arm_pose(&arm_pose)) {
    RCLCPP_WARN(node_->get_logger(), "[nav] mission command: no /navigation/state received yet.");
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] mission pose source: lidar(map)=(%.3f, %.3f, %.3f) arm(map)=(%.3f, %.3f, %.3f)",
    lidar_pose.x, lidar_pose.y, lidar_pose.yaw,
    arm_pose.x, arm_pose.y, arm_pose.yaw);

  arm2_task::task::RelativePlanarPose relative_pose;
  if (!compute_command_relative_pose(command, &relative_pose)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] mission command: failed to compute arm-relative target pose.");
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] arm_to_task[%s id=%d]: x=%.3f y=%.3f yaw=%.3f dist=%.3f",
    nav_pose_tracker_->has_received_task_points() ? "navigation/task_points" : "fallback_static_points",
    relative_pose.task_pose.id,
    relative_pose.x,
    relative_pose.y,
    relative_pose.yaw,
    relative_pose.distance);
}

bool NavTaskInterface::compute_command_relative_pose(
  const MissionCommand & command,
  RelativePlanarPose * relative_pose)
{
  if (nav_pose_tracker_ == nullptr || relative_pose == nullptr) {
    return false;
  }

  arm2_task::task::PlanarPose target_pose;
  const int task_point_id = static_cast<int>(command.task_index);
  if (!nav_pose_tracker_->get_task_point_pose(task_point_id, &target_pose)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] task_index=%u not found in /navigation/task_points or fallback task_nav static points.",
      command.task_index);
    return false;
  }

  if (std::abs(command.x) > 1e-9 || std::abs(command.y) > 1e-9) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[nav] mission task_index=%u uses /navigation/task_points id=%d map=(%.3f, %.3f); ignoring service xy=(%.3f, %.3f).",
      command.task_index, target_pose.id, target_pose.x, target_pose.y, command.x, command.y);
  }

  return nav_pose_tracker_->compute_relative_pose_to_task_pose(target_pose, relative_pose);
}

void NavTaskInterface::send_nav_event(const std::string & event)
{
  navigation::srv::StringCommand::Request req_msg;
  req_msg.message = event;
  publish_nav_event_request_debug(req_msg);

  if (!nav_event_client_->wait_for_service(std::chrono::seconds(1))) {
    navigation::srv::StringCommand::Response debug_resp;
    debug_resp.success = false;
    debug_resp.message = "service unavailable";
    publish_nav_event_response_debug(debug_resp);
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
    navigation::srv::StringCommand::Response debug_resp;
    debug_resp.success = false;
    debug_resp.message = "timed out";
    publish_nav_event_response_debug(debug_resp);
    RCLCPP_WARN(node_->get_logger(), "[nav] arm_event \"%s\" timed out.", event.c_str());
    return;
  }

  auto resp = future.get();
  if (!resp) {
    navigation::srv::StringCommand::Response debug_resp;
    debug_resp.success = false;
    debug_resp.message = "null response";
    publish_nav_event_response_debug(debug_resp);
    RCLCPP_WARN(node_->get_logger(), "[nav] arm_event \"%s\" returned null response.", event.c_str());
    return;
  }

  publish_nav_event_response_debug(*resp);
  if (resp->success) {
    RCLCPP_INFO(node_->get_logger(), "[nav] arm_event \"%s\" acknowledged.", event.c_str());
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "[nav] arm_event \"%s\" rejected: %s",
      event.c_str(), resp->message.c_str());
  }
}

bool NavTaskInterface::do_grasp_sequence(const MissionCommand & command)
{
  if (!do_pickup_lookout_align(command)) {
    return false;
  }
  clear_active_ready_command();

  if (!sequences_->grasp_from_current_view()) {
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

void NavTaskInterface::cache_active_ready_command(const MissionCommand & command)
{
  std::lock_guard<std::mutex> lk(cmd_mutex_);
  active_ready_command_ = command;
}

void NavTaskInterface::clear_active_ready_command()
{
  std::lock_guard<std::mutex> lk(cmd_mutex_);
  active_ready_command_.reset();
}

std::optional<NavTaskInterface::MissionCommand> NavTaskInterface::active_ready_command_copy()
{
  std::lock_guard<std::mutex> lk(cmd_mutex_);
  return active_ready_command_;
}

bool NavTaskInterface::do_pickup_lookout_align(const MissionCommand & command)
{
  const auto ready_command = active_ready_command_copy();
  const auto & lookout_command = ready_command.has_value() ? *ready_command : command;
  if (!ready_command.has_value()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] Pickup has no cached ready target; falling back to pickup task_index=%u.",
      command.task_index);
  }

  arm2_task::task::RelativePlanarPose relative_pose;
  if (!compute_command_relative_pose(lookout_command, &relative_pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[nav] Pickup lookout failed: unable to compute radar-relative target pose for task_index=%u.",
      lookout_command.task_index);
    return false;
  }

  geometry_msgs::msg::Pose look_target;
  look_target.position.x = relative_pose.x;
  look_target.position.y = relative_pose.y;
  look_target.position.z = 0.0;
  look_target.orientation.w = 1.0;

  const double lookout_yaw = std::atan2(relative_pose.y, relative_pose.x);
  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] Pickup lookout task_index=%u target_id=%d arm=(x=%.3f, y=%.3f), joint0_yaw=%.3f rad (%.1f deg).",
    lookout_command.task_index,
    relative_pose.task_pose.id,
    relative_pose.x,
    relative_pose.y,
    lookout_yaw,
    lookout_yaw * 180.0 / M_PI);

  if (!primitives_->do_look_out(look_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[nav] Pickup lookout move failed.");
    return false;
  }
  state_ = arm2_task::TaskState::LOOKOUT;
  return true;
}

bool NavTaskInterface::do_ready_sequence(const MissionCommand & command)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] Ready cached for task_index=%u point_id=%d. Arm will keep current posture until pickup.",
    command.task_index, command.point_id);
  cache_active_ready_command(command);
  return true;
}

bool NavTaskInterface::do_place_sequence()
{
  if (!sequences_->place_from_perception()) {
    return false;
  }

  rclcpp::sleep_for(500ms);
  send_nav_event("placed");
  primitives_->do_reset();
  send_nav_event("completed");
  return true;
}

bool NavTaskInterface::execute_mission_command(const MissionCommand & command)
{
  log_mission_command(command);

  if (command.action == "ready") {
    clear_active_ready_command();
    if (state_ != arm2_task::TaskState::IDLE && state_ != arm2_task::TaskState::LOOKOUT) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] Ready command ignored because arm state is %d, expected IDLE/LOOKOUT.",
        static_cast<int>(state_));
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Caching ready target without arm motion...");
    return do_ready_sequence(command);
  }

  if (command.action == "pickup") {
    if (state_ != arm2_task::TaskState::IDLE && state_ != arm2_task::TaskState::LOOKOUT) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] Pickup command ignored because arm state is %d, expected IDLE/LOOKOUT.",
        static_cast<int>(state_));
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Executing pickup sequence...");
    if (do_grasp_sequence(command)) {
      state_ = arm2_task::TaskState::HOLDING;
      return true;
    }
    return false;
  }

  if (command.action == "place") {
    clear_active_ready_command();
    if (state_ != arm2_task::TaskState::HOLDING) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] Place command ignored because arm state is %d, expected HOLDING.",
        static_cast<int>(state_));
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Executing place sequence...");
    if (do_place_sequence()) {
      state_ = arm2_task::TaskState::IDLE;
      return true;
    }
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "[nav] Unknown mission action: %s", command.action.c_str());
  return false;
}

void NavTaskInterface::run()
{
  RCLCPP_INFO(node_->get_logger(), "[nav] Ready. Waiting for /arm/mission_event mission commands...");

  while (rclcpp::ok() && is_running_->load()) {
    MissionCommand command;
    {
      std::unique_lock<std::mutex> lk(cmd_mutex_);
      cmd_cv_.wait_for(
        lk,
        std::chrono::milliseconds(200),
        [this] { return pending_command_.has_value(); });
      if (!pending_command_) {
        continue;
      }
      command = *pending_command_;
      pending_command_.reset();
    }

    if (remote_busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "[nav] Mission command received but arm is busy, ignoring.");
      continue;
    }

    log_nav_pose_snapshot(command.action.c_str());
    execute_mission_command(command);
    remote_busy_.store(false);
  }
}

}  // namespace arm2_task::task
