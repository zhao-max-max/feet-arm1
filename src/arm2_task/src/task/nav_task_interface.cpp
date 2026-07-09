#include "arm2_task/task/nav_task_interface.hpp"

#include <chrono>
#include <cmath>
#include <sstream>

#include "arm2_task/task/timing_events.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

namespace arm2_task::task
{

NavTaskInterface::NavTaskInterface(
  rclcpp::Node * node,
  TaskSequences * sequences,
  TaskPrimitives * primitives,
  NavPoseTracker * nav_pose_tracker,
  const std::atomic<bool> * is_running,
  Config config)
: node_(node),
  sequences_(sequences),
  primitives_(primitives),
  nav_pose_tracker_(nav_pose_tracker),
  is_running_(is_running),
  config_(config)
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
  timing_event_pub_ = create_timing_event_publisher(node_);

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
          (request->action != "ready" && request->action != "pickup" &&
           request->action != "place" && request->action != "pickup2" &&
           request->action != "place2")) {
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
          current_state != arm2_task::TaskState::LOOKOUT &&
          current_state != arm2_task::TaskState::STORED) {
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
      if (request->action == "pickup2" &&
          current_state != arm2_task::TaskState::IDLE &&
          current_state != arm2_task::TaskState::LOOKOUT) {
        response->success = false;
        response->message = "arm is not idle";
        publish_mission_response_debug(*response);
        RCLCPP_WARN(
          node_->get_logger(),
          "[nav] Rejected pickup2 task_index=%u point_id=%d: arm state=%d, expected IDLE/LOOKOUT.",
          request->task_index, request->point_id, static_cast<int>(current_state));
        return;
      }
      if (request->action == "place2" &&
          current_state != arm2_task::TaskState::STORED &&
          current_state != arm2_task::TaskState::IDLE) {
        response->success = false;
        response->message = "arm is not idle or stored";
        publish_mission_response_debug(*response);
        RCLCPP_WARN(
          node_->get_logger(),
          "[nav] Rejected place2 task_index=%u point_id=%d: arm state=%d, expected STORED/IDLE.",
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

  debug_state_server_ = node_->create_service<navigation::srv::MissionCommand>(
    "/arm/debug_state_command",
    [this](
      const navigation::srv::MissionCommand::Request::SharedPtr request,
      navigation::srv::MissionCommand::Response::SharedPtr response)
    {
      if (!request) {
        response->success = false;
        response->message = "null debug state request";
        return;
      }

      if (remote_busy_.exchange(true)) {
        response->success = false;
        response->message = "arm busy";
        RCLCPP_WARN(
          node_->get_logger(),
          "[debug_state] Rejected %s task_index=%u: arm busy.",
          request->action.c_str(), request->task_index);
        return;
      }

      const std::string timing_detail =
        "action=" + request->action + ",task_index=" + std::to_string(request->task_index) +
        ",point_id=" + std::to_string(request->point_id);
      publish_timing_event(
        node_, timing_event_pub_, "debug", "debug_state_command", "begin", timing_detail);
      const bool ok = handle_debug_state_command(*request, response.get());
      publish_timing_event(
        node_, timing_event_pub_, "debug", "debug_state_command", "end",
        timing_detail + ",ok=" + timing_bool(ok) + ",message=" + response->message);
      remote_busy_.store(false);
      RCLCPP_INFO(
        node_->get_logger(),
        "[debug_state] %s task_index=%u -> %s: %s",
        request->action.c_str(), request->task_index,
        ok ? "ok" : "fail", response->message.c_str());
    });

  nav_event_client_ = node_->create_client<navigation::srv::StringCommand>(
    "/navigation/arm_event");

  RCLCPP_INFO(
    node_->get_logger(),
    "Nav integration ready: /arm/mission_event + /arm/debug_state_command servers, /navigation/arm_event client.");
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

const char * NavTaskInterface::task_state_name(arm2_task::TaskState state) const
{
  switch (state) {
    case arm2_task::TaskState::IDLE:
      return "IDLE";
    case arm2_task::TaskState::LOOKOUT:
      return "LOOKOUT";
    case arm2_task::TaskState::OVERLOOK:
      return "OVERLOOK";
    case arm2_task::TaskState::GRASPING:
      return "GRASPING";
    case arm2_task::TaskState::HOLDING:
      return "HOLDING";
    case arm2_task::TaskState::PLACING:
      return "PLACING";
    case arm2_task::TaskState::STORED:
      return "STORED";
    case arm2_task::TaskState::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

void NavTaskInterface::set_task_state(
  arm2_task::TaskState next_state,
  const std::string & reason)
{
  const auto previous_state = state_;
  state_ = next_state;

  std::ostringstream detail;
  detail << "from=" << task_state_name(previous_state)
         << ",to=" << task_state_name(next_state)
         << ",reason=" << reason;
  publish_timing_event(
    node_, timing_event_pub_, "state", "task_state", "instant", detail.str());
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
  const std::string timing_detail = "event=" + event;
  publish_timing_event(
    node_, timing_event_pub_, "service", "navigation_arm_event", "begin", timing_detail);

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
    publish_timing_event(
      node_, timing_event_pub_, "service", "navigation_arm_event", "end",
      timing_detail + ",ok=0,reason=service_unavailable");
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
    publish_timing_event(
      node_, timing_event_pub_, "service", "navigation_arm_event", "end",
      timing_detail + ",ok=0,reason=timeout");
    return;
  }

  auto resp = future.get();
  if (!resp) {
    navigation::srv::StringCommand::Response debug_resp;
    debug_resp.success = false;
    debug_resp.message = "null response";
    publish_nav_event_response_debug(debug_resp);
    RCLCPP_WARN(node_->get_logger(), "[nav] arm_event \"%s\" returned null response.", event.c_str());
    publish_timing_event(
      node_, timing_event_pub_, "service", "navigation_arm_event", "end",
      timing_detail + ",ok=0,reason=null_response");
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
  publish_timing_event(
    node_, timing_event_pub_, "service", "navigation_arm_event", "end",
    timing_detail + ",ok=" + timing_bool(resp->success) + ",message=" + resp->message);
}

bool NavTaskInterface::do_grasp_sequence(const MissionCommand & command)
{
  std::ostringstream detail;
  detail << "task_index=" << command.task_index << ",point_id=" << command.point_id;
  publish_timing_event(node_, timing_event_pub_, "mission", "pickup_sequence", "begin", detail.str());

  if (!do_pickup_lookout_align(command)) {
    publish_timing_event(
      node_, timing_event_pub_, "mission", "pickup_sequence", "end",
      detail.str() + ",ok=0,reason=lookout_align_failed");
    return false;
  }
  clear_active_ready_command();

  if (!sequences_->grasp_from_current_view()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] Visual pickup failed for task_index=%u; trying radar fallback=%s.",
      command.task_index,
      config_.radar_pick_fallback_enabled ? "enabled" : "disabled");
    if (!do_radar_pick_fallback(command)) {
      publish_timing_event(
        node_, timing_event_pub_, "mission", "pickup_sequence", "end",
        detail.str() + ",ok=0,reason=grasp_failed");
      return false;
    }
  }

  rclcpp::sleep_for(500ms);
  send_nav_event("grabbed");

  if (!sequences_->move_to_carry_loaded()) {
    publish_timing_event(
      node_, timing_event_pub_, "mission", "pickup_sequence", "end",
      detail.str() + ",ok=0,reason=carry_move_failed");
    return false;
  }

  send_nav_event("completed");
  publish_timing_event(
    node_, timing_event_pub_, "mission", "pickup_sequence", "end",
    detail.str() + ",ok=1");
  return true;
}

bool NavTaskInterface::do_radar_pick_fallback(const MissionCommand & command)
{
  std::ostringstream detail;
  detail << "task_index=" << command.task_index
         << ",point_id=" << command.point_id
         << ",enabled=" << timing_bool(config_.radar_pick_fallback_enabled);
  publish_timing_event(
    node_, timing_event_pub_, "mission", "radar_pick_fallback", "begin", detail.str());

  if (!config_.radar_pick_fallback_enabled) {
    publish_timing_event(
      node_, timing_event_pub_, "mission", "radar_pick_fallback", "end",
      detail.str() + ",ok=0,reason=disabled");
    return false;
  }

  arm2_task::task::RelativePlanarPose relative_pose;
  if (!compute_command_relative_pose(command, &relative_pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[nav] Radar pickup fallback failed: unable to compute arm-relative pose for task_index=%u.",
      command.task_index);
    publish_timing_event(
      node_, timing_event_pub_, "mission", "radar_pick_fallback", "end",
      detail.str() + ",ok=0,reason=relative_pose_failed");
    return false;
  }

  geometry_msgs::msg::Pose fallback_pose;
  fallback_pose.position.x = relative_pose.x;
  fallback_pose.position.y = relative_pose.y;
  fallback_pose.position.z = config_.radar_pick_fallback_target_z;
  fallback_pose.orientation.x = 0.0;
  fallback_pose.orientation.y = 0.0;
  fallback_pose.orientation.z = std::sin(relative_pose.yaw / 2.0);
  fallback_pose.orientation.w = std::cos(relative_pose.yaw / 2.0);

  RCLCPP_WARN(
    node_->get_logger(),
    "[nav] Radar pickup fallback target_id=%d arm=(x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad).",
    relative_pose.task_pose.id,
    fallback_pose.position.x,
    fallback_pose.position.y,
    fallback_pose.position.z,
    relative_pose.yaw);

  if (!sequences_->grasp_pose(fallback_pose, false)) {
    publish_timing_event(
      node_, timing_event_pub_, "mission", "radar_pick_fallback", "end",
      detail.str() + ",ok=0,reason=fallback_grasp_failed,target_id=" +
      std::to_string(relative_pose.task_pose.id));
    return false;
  }

  publish_timing_event(
    node_, timing_event_pub_, "mission", "radar_pick_fallback", "end",
    detail.str() + ",ok=1,target_id=" + std::to_string(relative_pose.task_pose.id) +
    ",rel_x=" + std::to_string(relative_pose.x) +
    ",rel_y=" + std::to_string(relative_pose.y) +
    ",target_z=" + std::to_string(fallback_pose.position.z));
  return true;
}

bool NavTaskInterface::do_lookout_align_for_command(
  const MissionCommand & command,
  const char * context)
{
  std::ostringstream detail;
  detail << "context=" << context
         << ",task_index=" << command.task_index
         << ",point_id=" << command.point_id;
  publish_timing_event(
    node_, timing_event_pub_, "action", "nav_lookout_align", "begin", detail.str());

  arm2_task::task::RelativePlanarPose relative_pose;
  if (!compute_command_relative_pose(command, &relative_pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[nav] %s lookout failed: unable to compute radar-relative target pose for task_index=%u.",
      context, command.task_index);
    publish_timing_event(
      node_, timing_event_pub_, "action", "nav_lookout_align", "end",
      detail.str() + ",ok=0,reason=relative_pose_failed");
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
    "[nav] %s lookout task_index=%u target_id=%d arm=(x=%.3f, y=%.3f), joint0_yaw=%.3f rad (%.1f deg).",
    context,
    command.task_index,
    relative_pose.task_pose.id,
    relative_pose.x,
    relative_pose.y,
    lookout_yaw,
    lookout_yaw * 180.0 / M_PI);

  if (!primitives_->do_look_out(look_target)) {
    RCLCPP_ERROR(node_->get_logger(), "[nav] %s lookout move failed.", context);
    publish_timing_event(
      node_, timing_event_pub_, "action", "nav_lookout_align", "end",
      detail.str() + ",ok=0,reason=lookout_move_failed");
    return false;
  }
  set_task_state(arm2_task::TaskState::LOOKOUT, std::string(context) + "_lookout_align");
  publish_timing_event(
    node_, timing_event_pub_, "action", "nav_lookout_align", "end",
    detail.str() + ",ok=1,target_id=" + std::to_string(relative_pose.task_pose.id) +
    ",rel_x=" + std::to_string(relative_pose.x) +
    ",rel_y=" + std::to_string(relative_pose.y) +
    ",yaw=" + std::to_string(lookout_yaw));
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

  return do_lookout_align_for_command(lookout_command, "Pickup");
}

bool NavTaskInterface::handle_debug_state_command(
  const navigation::srv::MissionCommand::Request & request,
  navigation::srv::MissionCommand::Response * response)
{
  if (response == nullptr) {
    return false;
  }

  MissionCommand command;
  command.task_index = request.task_index;
  command.point_id = request.point_id;
  command.action = request.action;
  command.x = request.x;
  command.y = request.y;

  if (command.action == "idle") {
    clear_active_ready_command();
    set_task_state(arm2_task::TaskState::IDLE, "debug_idle");
    response->success = true;
    response->message = "state set to IDLE; arm motion unchanged";
    return true;
  }

  if (command.action == "holding") {
    clear_active_ready_command();
    set_task_state(arm2_task::TaskState::HOLDING, "debug_holding");
    response->success = true;
    response->message = "state set to HOLDING; arm motion unchanged";
    return true;
  }

  if (command.action == "carry_holding") {
    clear_active_ready_command();
    if (!sequences_->move_to_carry_loaded()) {
      response->success = false;
      response->message = "failed to move to carry loaded";
      return false;
    }
    set_task_state(arm2_task::TaskState::HOLDING, "debug_carry_holding");
    response->success = true;
    response->message = "moved to carry and set HOLDING";
    return true;
  }

  if (command.action == "lookout") {
    if (command.task_index == 0) {
      const auto ready_command = active_ready_command_copy();
      if (!ready_command.has_value()) {
        response->success = false;
        response->message = "lookout requires task_index or cached ready target";
        return false;
      }
      command = *ready_command;
      command.action = "lookout";
    }

    if (!do_lookout_align_for_command(command, "Debug")) {
      response->success = false;
      response->message = "lookout align failed";
      return false;
    }
    response->success = true;
    response->message = "state set to LOOKOUT";
    return true;
  }

  response->success = false;
  response->message = "invalid debug action; expected idle/lookout/holding/carry_holding";
  return false;
}

bool NavTaskInterface::do_ready_sequence(const MissionCommand & command)
{
  std::ostringstream detail;
  detail << "task_index=" << command.task_index << ",point_id=" << command.point_id;
  publish_timing_event(node_, timing_event_pub_, "mission", "ready_sequence", "begin", detail.str());
  RCLCPP_INFO(
    node_->get_logger(),
    "[nav] Ready cached for task_index=%u point_id=%d. Arm will keep current posture until pickup.",
    command.task_index, command.point_id);
  cache_active_ready_command(command);
  publish_timing_event(
    node_, timing_event_pub_, "mission", "ready_sequence", "end", detail.str() + ",ok=1");
  return true;
}

bool NavTaskInterface::do_place_sequence(const MissionCommand & command){
  std::ostringstream detail;
  detail << "task_index=" << command.task_index
         << ",point_id=" << command.point_id
         << ",place_height=" << config_.place_height;
  publish_timing_event(node_, timing_event_pub_, "mission", "place_sequence", "begin", detail.str());

  // 先尝试狗头相机感知放置（含 roll 对齐）
  bool place_ok = sequences_->place_from_perception();

  if (!place_ok) {
    // 感知失败，降级到雷达坐标兜底
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] Visual place failed for task_index=%u; falling back to radar pose.",
      command.task_index);

    arm2_task::task::RelativePlanarPose relative_pose;
    if (!compute_command_relative_pose(command, &relative_pose)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[nav] Place fallback failed: unable to compute arm-relative pose for task_index=%u.",
        command.task_index);
      publish_timing_event(
        node_, timing_event_pub_, "mission", "place_sequence", "end",
        detail.str() + ",ok=0,reason=relative_pose_failed");
      return false;
    }

    geometry_msgs::msg::Pose fallback_pose;
    fallback_pose.position.x = relative_pose.x;
    fallback_pose.position.y = relative_pose.y;
    fallback_pose.position.z = config_.place_height;
    fallback_pose.orientation.x = 0.0;
    fallback_pose.orientation.y = 0.0;
    fallback_pose.orientation.z = std::sin(relative_pose.yaw / 2.0);
    fallback_pose.orientation.w = std::cos(relative_pose.yaw / 2.0);

    RCLCPP_INFO(
      node_->get_logger(),
      "[nav] Place fallback target_id=%d arm=(x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad).",
      relative_pose.task_pose.id,
      fallback_pose.position.x,
      fallback_pose.position.y,
      fallback_pose.position.z,
      relative_pose.yaw);

    place_ok = sequences_->place_pose_direct_height(fallback_pose);
    if (!place_ok) {
      publish_timing_event(
        node_, timing_event_pub_, "mission", "place_sequence", "end",
        detail.str() + ",ok=0,reason=place_fallback_failed,target_id=" +
        std::to_string(relative_pose.task_pose.id));
      return false;
    }
  }

  primitives_->do_reset();
  rclcpp::sleep_for(500ms);
  send_nav_event("placed");
  send_nav_event("completed");
  publish_timing_event(
    node_, timing_event_pub_, "mission", "place_sequence", "end",
    detail.str() + ",ok=1");
  return true;
}

bool NavTaskInterface::do_store_sequence(const MissionCommand & command)
{
  std::ostringstream detail;
  detail << "task_index=" << command.task_index << ",point_id=" << command.point_id;
  publish_timing_event(node_, timing_event_pub_, "mission", "store_sequence", "begin", detail.str());

  if (!do_pickup_lookout_align(command)) {
    publish_timing_event(
      node_, timing_event_pub_, "mission", "store_sequence", "end",
      detail.str() + ",ok=0,reason=lookout_align_failed");
    return false;
  }
  clear_active_ready_command();

  // Visual grasp + radar fallback (same pattern as do_grasp_sequence)
  bool grasp_ok = sequences_->grasp_from_current_view();
  if (!grasp_ok) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] pickup2 visual grasp failed for task_index=%u; trying radar fallback.",
      command.task_index);
    if (!do_radar_pick_fallback(command)) {
      publish_timing_event(
        node_, timing_event_pub_, "mission", "store_sequence", "end",
        detail.str() + ",ok=0,reason=grasp_failed");
      return false;
    }
  }

  // Hand off box to dog suction cup
  sequences_->handoff_to_dog();

  send_nav_event("stored");
  send_nav_event("completed");
  publish_timing_event(
    node_, timing_event_pub_, "mission", "store_sequence", "end",
    detail.str() + ",ok=1");
  return true;
}

bool NavTaskInterface::do_pickup_from_dog_sequence(const MissionCommand & command)
{
  std::ostringstream detail;
  detail << "task_index=" << command.task_index << ",point_id=" << command.point_id;
  publish_timing_event(
    node_, timing_event_pub_, "mission", "pickup_from_dog_sequence", "begin", detail.str());

  // Step 1: move to store pose, release dog suction, arm takes box
  if (!sequences_->pickup_from_dog()) {
    publish_timing_event(
      node_, timing_event_pub_, "mission", "pickup_from_dog_sequence", "end",
      detail.str() + ",ok=0,reason=pickup_from_dog_failed");
    return false;
  }

  // Step 2: place with visual perception + radar fallback (same as do_place_sequence)
  bool place_ok = sequences_->place_from_perception();

  if (!place_ok) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[nav] place2 visual place failed for task_index=%u; falling back to radar pose.",
      command.task_index);

    arm2_task::task::RelativePlanarPose relative_pose;
    if (!compute_command_relative_pose(command, &relative_pose)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[nav] place2 fallback failed: unable to compute arm-relative pose for task_index=%u.",
        command.task_index);
      publish_timing_event(
        node_, timing_event_pub_, "mission", "pickup_from_dog_sequence", "end",
        detail.str() + ",ok=0,reason=fallback_relative_pose_failed");
      primitives_->do_reset();
      primitives_->do_dog_suction_on();
      return false;
    }

    geometry_msgs::msg::Pose fallback_pose;
    fallback_pose.position.x = relative_pose.x;
    fallback_pose.position.y = relative_pose.y;
    fallback_pose.position.z = config_.place_height;
    fallback_pose.orientation.x = 0.0;
    fallback_pose.orientation.y = 0.0;
    fallback_pose.orientation.z = std::sin(relative_pose.yaw / 2.0);
    fallback_pose.orientation.w = std::cos(relative_pose.yaw / 2.0);

    place_ok = sequences_->place_pose_direct_height(fallback_pose);
    if (!place_ok) {
      publish_timing_event(
        node_, timing_event_pub_, "mission", "pickup_from_dog_sequence", "end",
        detail.str() + ",ok=0,reason=fallback_place_failed");
      primitives_->do_reset();
      primitives_->do_dog_suction_on();
      return false;
    }
  }

  primitives_->do_reset();
  primitives_->do_dog_suction_on();

  rclcpp::sleep_for(500ms);
  send_nav_event("placed");
  send_nav_event("completed");
  publish_timing_event(
    node_, timing_event_pub_, "mission", "pickup_from_dog_sequence", "end",
    detail.str() + ",ok=1");
  return true;
}

bool NavTaskInterface::execute_mission_command(const MissionCommand & command)
{
  log_mission_command(command);
  std::ostringstream detail;
  detail << "action=" << command.action
         << ",task_index=" << command.task_index
         << ",point_id=" << command.point_id;
  publish_timing_event(node_, timing_event_pub_, "mission", "execute_command", "begin", detail.str());

  if (command.action == "ready") {
    clear_active_ready_command();
    if (state_ != arm2_task::TaskState::IDLE && state_ != arm2_task::TaskState::LOOKOUT) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] Ready command ignored because arm state is %d, expected IDLE/LOOKOUT.",
        static_cast<int>(state_));
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=0,reason=invalid_state");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Caching ready target without arm motion...");
    const bool ok = do_ready_sequence(command);
    publish_timing_event(
      node_, timing_event_pub_, "mission", "execute_command", "end",
      detail.str() + ",ok=" + timing_bool(ok));
    return ok;
  }

  if (command.action == "pickup") {
    if (state_ != arm2_task::TaskState::IDLE &&
        state_ != arm2_task::TaskState::LOOKOUT &&
        state_ != arm2_task::TaskState::STORED) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] Pickup command ignored because arm state is %d, expected IDLE/LOOKOUT.",
        static_cast<int>(state_));
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=0,reason=invalid_state");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Executing pickup sequence...");
    if (do_grasp_sequence(command)) {
      set_task_state(arm2_task::TaskState::HOLDING, "pickup_complete");
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=1");
      return true;
    }
    publish_timing_event(
      node_, timing_event_pub_, "mission", "execute_command", "end",
      detail.str() + ",ok=0,reason=pickup_sequence_failed");
    return false;
  }

  if (command.action == "place") {
    clear_active_ready_command();
    if (state_ != arm2_task::TaskState::HOLDING) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] Place command ignored because arm state is %d, expected HOLDING.",
        static_cast<int>(state_));
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=0,reason=invalid_state");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Executing place sequence...");
    if (do_place_sequence(command)) {
      set_task_state(arm2_task::TaskState::IDLE, "place_complete");
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=1");
      return true;
    }
    publish_timing_event(
      node_, timing_event_pub_, "mission", "execute_command", "end",
      detail.str() + ",ok=0,reason=place_sequence_failed");
    return false;
  }

  if (command.action == "pickup2") {
    if (state_ != arm2_task::TaskState::IDLE && state_ != arm2_task::TaskState::LOOKOUT) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] pickup2 ignored because arm state is %d, expected IDLE/LOOKOUT.",
        static_cast<int>(state_));
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=0,reason=invalid_state");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Executing store sequence (pickup2)...");
    if (do_store_sequence(command)) {
      set_task_state(arm2_task::TaskState::STORED, "pickup2_complete");
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=1");
      return true;
    }
    publish_timing_event(
      node_, timing_event_pub_, "mission", "execute_command", "end",
      detail.str() + ",ok=0,reason=store_sequence_failed");
    return false;
  }

  if (command.action == "place2") {
    if (state_ != arm2_task::TaskState::STORED &&
        state_ != arm2_task::TaskState::IDLE) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[nav] place2 ignored because arm state is %d, expected STORED/IDLE.",
        static_cast<int>(state_));
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=0,reason=invalid_state");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[nav] Executing pickup-from-dog-and-place sequence (place2)...");
    if (do_pickup_from_dog_sequence(command)) {
      set_task_state(arm2_task::TaskState::IDLE, "place2_complete");
      publish_timing_event(
        node_, timing_event_pub_, "mission", "execute_command", "end",
        detail.str() + ",ok=1");
      return true;
    }
    publish_timing_event(
      node_, timing_event_pub_, "mission", "execute_command", "end",
      detail.str() + ",ok=0,reason=pickup_from_dog_sequence_failed");
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "[nav] Unknown mission action: %s", command.action.c_str());
  publish_timing_event(
    node_, timing_event_pub_, "mission", "execute_command", "end",
    detail.str() + ",ok=0,reason=unknown_action");
  return false;
}

void NavTaskInterface::run()
{
  RCLCPP_INFO(node_->get_logger(), "[nav] Ready. Homing to reset posture...");
  primitives_->do_reset();
  RCLCPP_INFO(node_->get_logger(), "[nav] Waiting for /arm/mission_event mission commands...");

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
