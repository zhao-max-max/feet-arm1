#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "navigation/msg/map_point_array.hpp"
#include "navigation/srv/mission_command.hpp"
#include "navigation/srv/string_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_joint.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/get_payload_estimate.hpp"
#include "robot_msgs/srv/get_pick_pos.hpp"
#include "robot_msgs/srv/get_place_pos.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "robot_msgs/srv/set_payload_state.hpp"
#include "robot_msgs/srv/set_suction.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

const char * ready_word(bool ready)
{
  return ready ? "up" : "down";
}

std::string bool_text(bool value)
{
  return value ? "1" : "0";
}

std::string csv_escape(const std::string & value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }

  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const char ch : value) {
    if (ch == '"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(ch);
    }
  }
  escaped.push_back('"');
  return escaped;
}

double yaw_from_quaternion_wxyz(double w, double x, double y, double z)
{
  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z));
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class DebugToolNode : public rclcpp::Node
{
public:
  using MoveJoint = robot_msgs::action::MoveJoint;

  DebugToolNode()
  : Node("debug_tool_node")
  {
    start_wall_time_ = std::chrono::system_clock::now();
    start_ros_time_ = now();

    state_topic_ = declare_parameter<std::string>("state_topic", "/arm2/_lowState/joint");
    command_topic_ = declare_parameter<std::string>("command_topic", "/arm2/_lowCmd/command");
    ready_topic_ = declare_parameter<std::string>("ready_topic", "/robot_driver/ready");
    nav_state_topic_ = declare_parameter<std::string>("nav_state_topic", "/navigation/state");
    nav_task_points_topic_ =
      declare_parameter<std::string>("nav_task_points_topic", "/navigation/task_points");
    arm_mission_service_ =
      declare_parameter<std::string>("arm_mission_service", "/arm/mission_event");
    nav_arm_event_service_ =
      declare_parameter<std::string>("nav_arm_event_service", "/navigation/arm_event");
    nav_mission_request_topic_ =
      declare_parameter<std::string>("nav_mission_request_topic", "/debug/nav/mission_request");
    nav_mission_response_topic_ =
      declare_parameter<std::string>("nav_mission_response_topic", "/debug/nav/mission_response");
    nav_arm_event_request_topic_ =
      declare_parameter<std::string>(
      "nav_arm_event_request_topic", "/debug/nav/arm_event_request");
    nav_arm_event_response_topic_ =
      declare_parameter<std::string>(
      "nav_arm_event_response_topic", "/debug/nav/arm_event_response");
    motor_count_ = declare_parameter<int>("motor_count", 5);
    csv_enabled_ = declare_parameter<bool>("csv_enabled", true);
    csv_dir_ = declare_parameter<std::string>("csv_dir", "debug_tool_logs");
    const double report_period_sec = std::max(
      0.1, declare_parameter<double>("report_period_sec", 0.05));

    open_csv_file();

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = create_subscription<robot_msgs::msg::RobotState>(
      state_topic_, state_qos,
      [this](const robot_msgs::msg::RobotState::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = *msg;
        last_state_time_ = now();
        has_state_ = true;
      });

    command_sub_ = create_subscription<robot_msgs::msg::RobotCommand>(
      command_topic_, state_qos,
      [this](const robot_msgs::msg::RobotCommand::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_command_ = *msg;
        last_command_time_ = now();
        has_command_ = true;
      });

    auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      ready_topic_, ready_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        driver_ready_ = msg->data;
        last_ready_time_ = now();
        has_ready_ = true;
      });

    nav_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      nav_state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_nav_state_ = *msg;
        last_nav_state_time_ = now();
        has_nav_state_ = true;
      });

    auto nav_task_points_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    nav_task_points_sub_ = create_subscription<navigation::msg::MapPointArray>(
      nav_task_points_topic_, nav_task_points_qos,
      [this](const navigation::msg::MapPointArray::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_nav_task_points_ = *msg;
        last_nav_task_points_time_ = now();
        has_nav_task_points_ = true;
      });

    auto nav_debug_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    nav_mission_request_sub_ =
      create_subscription<navigation::srv::MissionCommand::Request>(
      nav_mission_request_topic_, nav_debug_qos,
      [this](const navigation::srv::MissionCommand::Request::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_nav_mission_request_ = *msg;
        last_nav_mission_request_time_ = now();
        has_nav_mission_request_ = true;
      });

    nav_mission_response_sub_ =
      create_subscription<navigation::srv::MissionCommand::Response>(
      nav_mission_response_topic_, nav_debug_qos,
      [this](const navigation::srv::MissionCommand::Response::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_nav_mission_response_ = *msg;
        last_nav_mission_response_time_ = now();
        has_nav_mission_response_ = true;
      });

    nav_arm_event_request_sub_ =
      create_subscription<navigation::srv::StringCommand::Request>(
      nav_arm_event_request_topic_, nav_debug_qos,
      [this](const navigation::srv::StringCommand::Request::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_nav_arm_event_request_ = *msg;
        last_nav_arm_event_request_time_ = now();
        has_nav_arm_event_request_ = true;
      });

    nav_arm_event_response_sub_ =
      create_subscription<navigation::srv::StringCommand::Response>(
      nav_arm_event_response_topic_, nav_debug_qos,
      [this](const navigation::srv::StringCommand::Response::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_nav_arm_event_response_ = *msg;
        last_nav_arm_event_response_time_ = now();
        has_nav_arm_event_response_ = true;
      });

    mode_client_ = create_client<robot_msgs::srv::SetControllerMode>("set_controller_mode");
    suction_client_ = create_client<robot_msgs::srv::SetSuction>("set_suction");
    pick_client_ = create_client<robot_msgs::srv::GetPickPos>("get_pick_pos");
    place_client_ = create_client<robot_msgs::srv::GetPlacePos>("get_place_pos");
    stack_client_ = create_client<robot_msgs::srv::GetPlacePos>("get_stack_pos");
    payload_estimate_client_ =
      create_client<robot_msgs::srv::GetPayloadEstimate>("get_payload_estimate");
    payload_state_client_ =
      create_client<robot_msgs::srv::SetPayloadState>("set_payload_state");
    arm_mission_client_ =
      create_client<navigation::srv::MissionCommand>(arm_mission_service_);
    nav_arm_event_client_ =
      create_client<navigation::srv::StringCommand>(nav_arm_event_service_);
    move_joint_client_ = rclcpp_action::create_client<MoveJoint>(this, "move_joint");

    report_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(report_period_sec)),
      std::bind(&DebugToolNode::report, this));

    RCLCPP_INFO(
      get_logger(),
      "debug_tool_node started. state_topic=%s command_topic=%s ready_topic=%s "
      "nav_state_topic=%s nav_task_points_topic=%s arm_mission_service=%s "
      "nav_arm_event_service=%s nav_mission_request_topic=%s "
      "nav_mission_response_topic=%s nav_arm_event_request_topic=%s "
      "nav_arm_event_response_topic=%s report_period=%.2fs csv=%s",
      state_topic_.c_str(), command_topic_.c_str(), ready_topic_.c_str(),
      nav_state_topic_.c_str(), nav_task_points_topic_.c_str(), arm_mission_service_.c_str(),
      nav_arm_event_service_.c_str(), nav_mission_request_topic_.c_str(),
      nav_mission_response_topic_.c_str(), nav_arm_event_request_topic_.c_str(),
      nav_arm_event_response_topic_.c_str(), report_period_sec,
      csv_path_.empty() ? "disabled" : csv_path_.c_str());
  }

private:
  struct ServiceState
  {
    bool mode{false};
    bool suction{false};
    bool pick{false};
    bool place{false};
    bool stack{false};
    bool payload_estimate{false};
    bool payload_state{false};
    bool arm_mission{false};
    bool nav_arm_event{false};
  };

  void report()
  {
    robot_msgs::msg::RobotState state_snapshot;
    robot_msgs::msg::RobotCommand command_snapshot;
    nav_msgs::msg::Odometry nav_state_snapshot;
    navigation::msg::MapPointArray nav_task_points_snapshot;
    navigation::srv::MissionCommand::Request nav_mission_request_snapshot;
    navigation::srv::MissionCommand::Response nav_mission_response_snapshot;
    navigation::srv::StringCommand::Request nav_arm_event_request_snapshot;
    navigation::srv::StringCommand::Response nav_arm_event_response_snapshot;
    rclcpp::Time state_time;
    rclcpp::Time command_time;
    rclcpp::Time ready_time;
    rclcpp::Time nav_state_time;
    rclcpp::Time nav_task_points_time;
    rclcpp::Time nav_mission_request_time;
    rclcpp::Time nav_mission_response_time;
    rclcpp::Time nav_arm_event_request_time;
    rclcpp::Time nav_arm_event_response_time;
    bool has_state = false;
    bool has_command = false;
    bool has_ready = false;
    bool has_nav_state = false;
    bool has_nav_task_points = false;
    bool has_nav_mission_request = false;
    bool has_nav_mission_response = false;
    bool has_nav_arm_event_request = false;
    bool has_nav_arm_event_response = false;
    bool driver_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_snapshot = last_state_;
      command_snapshot = last_command_;
      nav_state_snapshot = last_nav_state_;
      nav_task_points_snapshot = last_nav_task_points_;
      nav_mission_request_snapshot = last_nav_mission_request_;
      nav_mission_response_snapshot = last_nav_mission_response_;
      nav_arm_event_request_snapshot = last_nav_arm_event_request_;
      nav_arm_event_response_snapshot = last_nav_arm_event_response_;
      state_time = last_state_time_;
      command_time = last_command_time_;
      ready_time = last_ready_time_;
      nav_state_time = last_nav_state_time_;
      nav_task_points_time = last_nav_task_points_time_;
      nav_mission_request_time = last_nav_mission_request_time_;
      nav_mission_response_time = last_nav_mission_response_time_;
      nav_arm_event_request_time = last_nav_arm_event_request_time_;
      nav_arm_event_response_time = last_nav_arm_event_response_time_;
      has_state = has_state_;
      has_command = has_command_;
      has_ready = has_ready_;
      has_nav_state = has_nav_state_;
      has_nav_task_points = has_nav_task_points_;
      has_nav_mission_request = has_nav_mission_request_;
      has_nav_mission_response = has_nav_mission_response_;
      has_nav_arm_event_request = has_nav_arm_event_request_;
      has_nav_arm_event_response = has_nav_arm_event_response_;
      driver_ready = driver_ready_;
    }

    const auto now_time = now();
    const double state_age = has_state ? (now_time - state_time).seconds() : -1.0;
    const double command_age = has_command ? (now_time - command_time).seconds() : -1.0;
    const double ready_age = has_ready ? (now_time - ready_time).seconds() : -1.0;
    const double nav_state_age = has_nav_state ? (now_time - nav_state_time).seconds() : -1.0;
    const double nav_task_points_age =
      has_nav_task_points ? (now_time - nav_task_points_time).seconds() : -1.0;
    const double nav_mission_request_age =
      has_nav_mission_request ? (now_time - nav_mission_request_time).seconds() : -1.0;
    const double nav_mission_response_age =
      has_nav_mission_response ? (now_time - nav_mission_response_time).seconds() : -1.0;
    const double nav_arm_event_request_age =
      has_nav_arm_event_request ? (now_time - nav_arm_event_request_time).seconds() : -1.0;
    const double nav_arm_event_response_age =
      has_nav_arm_event_response ? (now_time - nav_arm_event_response_time).seconds() : -1.0;
    const auto service_state = read_service_state();
    const bool action_ready = move_joint_client_->action_server_is_ready();
    const auto nav_task_points_range = task_point_id_range(nav_task_points_snapshot);

    write_csv_row(
      now_time, state_snapshot, command_snapshot, has_state, has_command, has_ready,
      driver_ready, state_age, command_age, ready_age,
      nav_state_snapshot, has_nav_state, nav_state_age,
      nav_task_points_snapshot, has_nav_task_points, nav_task_points_age, nav_task_points_range,
      nav_mission_request_snapshot, has_nav_mission_request, nav_mission_request_age,
      nav_mission_response_snapshot, has_nav_mission_response, nav_mission_response_age,
      nav_arm_event_request_snapshot, has_nav_arm_event_request, nav_arm_event_request_age,
      nav_arm_event_response_snapshot, has_nav_arm_event_response, nav_arm_event_response_age,
      service_state, action_ready);

    if (!has_state) {
      RCLCPP_WARN(
        get_logger(),
        "ready=%s ready_age=%.2fs cmd_age=%.2fs | no robot state received yet | services=%s "
        "nav_state_age=%.2fs nav_state=%s nav_task_points_age=%.2fs nav_task_points=%s "
        "mission_req_age=%.2fs mission_req=%s mission_rsp_age=%.2fs mission_rsp=%s "
        "arm_event_req_age=%.2fs arm_event_req=%s arm_event_rsp_age=%.2fs arm_event_rsp=%s "
        "action=%s",
        driver_ready ? "true" : "false",
        ready_age,
        command_age,
        service_summary(service_state).c_str(),
        nav_state_age,
        format_nav_state(nav_state_snapshot, has_nav_state).c_str(),
        nav_task_points_age,
        format_task_points_summary(nav_task_points_snapshot, has_nav_task_points).c_str(),
        nav_mission_request_age,
        format_mission_request(nav_mission_request_snapshot, has_nav_mission_request).c_str(),
        nav_mission_response_age,
        format_mission_response(nav_mission_response_snapshot, has_nav_mission_response).c_str(),
        nav_arm_event_request_age,
        format_arm_event_request(nav_arm_event_request_snapshot, has_nav_arm_event_request).c_str(),
        nav_arm_event_response_age,
        format_arm_event_response(nav_arm_event_response_snapshot, has_nav_arm_event_response).c_str(),
        ready_word(action_ready));
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "ready=%s ready_age=%.2fs state_age=%.2fs cmd_age=%.2fs valid=%s q_deg=[%s] dq=[%s] "
      "cmd_q_deg=[%s] | services=%s nav_state_age=%.2fs nav_state=%s "
      "nav_task_points_age=%.2fs nav_task_points=%s mission_req_age=%.2fs mission_req=%s "
      "mission_rsp_age=%.2fs mission_rsp=%s arm_event_req_age=%.2fs arm_event_req=%s "
      "arm_event_rsp_age=%.2fs arm_event_rsp=%s action=%s",
      driver_ready ? "true" : "false",
      ready_age,
      state_age,
      command_age,
      format_valid(state_snapshot).c_str(),
      format_positions_deg(state_snapshot).c_str(),
      format_velocities(state_snapshot).c_str(),
      has_command ? format_command_positions_deg(command_snapshot).c_str() : "",
      service_summary(service_state).c_str(),
      nav_state_age,
      format_nav_state(nav_state_snapshot, has_nav_state).c_str(),
      nav_task_points_age,
      format_task_points_summary(nav_task_points_snapshot, has_nav_task_points).c_str(),
      nav_mission_request_age,
      format_mission_request(nav_mission_request_snapshot, has_nav_mission_request).c_str(),
      nav_mission_response_age,
      format_mission_response(nav_mission_response_snapshot, has_nav_mission_response).c_str(),
      nav_arm_event_request_age,
      format_arm_event_request(nav_arm_event_request_snapshot, has_nav_arm_event_request).c_str(),
      nav_arm_event_response_age,
      format_arm_event_response(nav_arm_event_response_snapshot, has_nav_arm_event_response).c_str(),
      ready_word(action_ready));
  }

  ServiceState read_service_state() const
  {
    ServiceState state;
    state.mode = mode_client_->service_is_ready();
    state.suction = suction_client_->service_is_ready();
    state.pick = pick_client_->service_is_ready();
    state.place = place_client_->service_is_ready();
    state.stack = stack_client_->service_is_ready();
    state.payload_estimate = payload_estimate_client_->service_is_ready();
    state.payload_state = payload_state_client_->service_is_ready();
    state.arm_mission = arm_mission_client_->service_is_ready();
    state.nav_arm_event = nav_arm_event_client_->service_is_ready();
    return state;
  }

  std::string service_summary(const ServiceState & state) const
  {
    std::ostringstream out;
    out << "mode:" << ready_word(state.mode)
        << " suction:" << ready_word(state.suction)
        << " pick:" << ready_word(state.pick)
        << " place:" << ready_word(state.place)
        << " stack:" << ready_word(state.stack)
        << " payload_est:" << ready_word(state.payload_estimate)
        << " payload_state:" << ready_word(state.payload_state)
        << " arm_mission:" << ready_word(state.arm_mission)
        << " nav_arm_event:" << ready_word(state.nav_arm_event);
    return out.str();
  }

  int clamped_motor_count(const robot_msgs::msg::RobotState & state) const
  {
    const int available = static_cast<int>(state.motor_state.size());
    return std::max(0, std::min(motor_count_, available));
  }

  int clamped_motor_count(const robot_msgs::msg::RobotCommand & command) const
  {
    const int available = static_cast<int>(command.motor_command.size());
    return std::max(0, std::min(motor_count_, available));
  }

  std::string format_positions_deg(const robot_msgs::msg::RobotState & state) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(1);
    const int count = clamped_motor_count(state);
    for (int i = 0; i < count; ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << state.motor_state[i].q * kRadToDeg;
    }
    return out.str();
  }

  std::string format_velocities(const robot_msgs::msg::RobotState & state) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    const int count = clamped_motor_count(state);
    for (int i = 0; i < count; ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << state.motor_state[i].dq;
    }
    return out.str();
  }

  std::string format_command_positions_deg(const robot_msgs::msg::RobotCommand & command) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(1);
    const int count = clamped_motor_count(command);
    for (int i = 0; i < count; ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << command.motor_command[i].q * kRadToDeg;
    }
    return out.str();
  }

  std::string format_valid(const robot_msgs::msg::RobotState & state) const
  {
    std::ostringstream out;
    const int count = clamped_motor_count(state);
    for (int i = 0; i < count; ++i) {
      if (i > 0) {
        out << ",";
      }
      out << (state.motor_state[i].valid ? "1" : "0");
    }
    if (static_cast<int>(state.motor_state.size()) < motor_count_) {
      out << " size=" << state.motor_state.size();
    }
    return out.str();
  }

  std::string format_nav_state(
    const nav_msgs::msg::Odometry & nav_state,
    bool has_nav_state) const
  {
    if (!has_nav_state) {
      return "down";
    }

    const auto & pose = nav_state.pose.pose;
    const double yaw = yaw_from_quaternion_wxyz(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);

    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "map=(" << pose.position.x << ", " << pose.position.y << ", " << yaw << ")";
    return out.str();
  }

  std::pair<int, int> task_point_id_range(
    const navigation::msg::MapPointArray & task_points) const
  {
    if (task_points.points.empty()) {
      return {0, 0};
    }

    int min_id = task_points.points.front().id;
    int max_id = task_points.points.front().id;
    for (const auto & point : task_points.points) {
      min_id = std::min(min_id, point.id);
      max_id = std::max(max_id, point.id);
    }
    return {min_id, max_id};
  }

  std::string format_task_points_summary(
    const navigation::msg::MapPointArray & task_points,
    bool has_task_points) const
  {
    if (!has_task_points) {
      return "down";
    }

    std::ostringstream out;
    out << task_points.points.size() << "pts";
    if (!task_points.points.empty()) {
      const auto range = task_point_id_range(task_points);
      out << "[id=" << range.first << ".." << range.second << "]";
    }
    return out.str();
  }

  std::string format_task_points_csv(
    const navigation::msg::MapPointArray & task_points,
    bool has_task_points) const
  {
    if (!has_task_points || task_points.points.empty()) {
      return "";
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < task_points.points.size(); ++i) {
      if (i > 0) {
        out << "|";
      }
      const auto & point = task_points.points[i];
      out << point.id << ":" << point.x << ":" << point.y;
    }
    return out.str();
  }

  std::string format_mission_request(
    const navigation::srv::MissionCommand::Request & request,
    bool has_request) const
  {
    if (!has_request) {
      return "down";
    }

    std::ostringstream out;
    out << request.action
        << " idx=" << request.task_index
        << " point=" << request.point_id
        << " map=(" << std::fixed << std::setprecision(3) << request.x << ", " << request.y << ")";
    return out.str();
  }

  std::string format_mission_response(
    const navigation::srv::MissionCommand::Response & response,
    bool has_response) const
  {
    if (!has_response) {
      return "down";
    }

    std::ostringstream out;
    out << (response.success ? "ok" : "fail") << ":" << response.message;
    return out.str();
  }

  std::string format_arm_event_request(
    const navigation::srv::StringCommand::Request & request,
    bool has_request) const
  {
    if (!has_request) {
      return "down";
    }
    return request.message;
  }

  std::string format_arm_event_response(
    const navigation::srv::StringCommand::Response & response,
    bool has_response) const
  {
    if (!has_response) {
      return "down";
    }

    std::ostringstream out;
    out << (response.success ? "ok" : "fail") << ":" << response.message;
    return out.str();
  }

  std::string make_start_timestamp() const
  {
    const std::time_t raw_time = std::chrono::system_clock::to_time_t(start_wall_time_);
    std::tm local_time{};
    localtime_r(&raw_time, &local_time);

    std::ostringstream out;
    out << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return out.str();
  }

  std::string wall_timestamp_now() const
  {
    const auto current_wall_time = std::chrono::system_clock::now();
    const std::time_t raw_time = std::chrono::system_clock::to_time_t(current_wall_time);
    std::tm local_time{};
    localtime_r(&raw_time, &local_time);

    const auto duration = current_wall_time.time_since_epoch();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;

    std::ostringstream out;
    out << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms;
    return out.str();
  }

  void open_csv_file()
  {
    if (!csv_enabled_) {
      return;
    }

    const std::filesystem::path dir(csv_dir_);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      RCLCPP_ERROR(
        get_logger(), "Failed to create CSV directory '%s': %s",
        dir.string().c_str(), ec.message().c_str());
      return;
    }

    const auto filename = make_start_timestamp() + ".csv";
    const auto path = dir / filename;
    csv_file_.open(path);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV log file: %s", path.string().c_str());
      return;
    }

    csv_path_ = path.string();
    write_csv_header();
    RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
  }

  void write_csv_header()
  {
    if (!csv_file_.is_open()) {
      return;
    }

    csv_file_
      << "wall_time,ros_time_sec,elapsed_sec,"
      << "has_ready,driver_ready,ready_age_sec,"
      << "has_state,state_age_sec,state_size,"
      << "has_command,command_age_sec,command_size,"
      << "service_mode,service_suction,service_pick,service_place,service_stack,"
      << "service_payload_estimate,service_payload_state,service_arm_mission,"
      << "service_nav_arm_event,action_move_joint,"
      << "has_nav_state,nav_state_age_sec,nav_state_x,nav_state_y,nav_state_yaw,"
      << "has_nav_task_points,nav_task_points_age_sec,nav_task_points_count,"
      << "nav_task_points_min_id,nav_task_points_max_id,nav_task_points_summary,"
      << "has_nav_mission_request,nav_mission_request_age_sec,nav_mission_action,"
      << "nav_mission_task_index,nav_mission_point_id,nav_mission_x,nav_mission_y,"
      << "has_nav_mission_response,nav_mission_response_age_sec,nav_mission_response_success,"
      << "nav_mission_response_message,"
      << "has_nav_arm_event_request,nav_arm_event_request_age_sec,nav_arm_event_request_message,"
      << "has_nav_arm_event_response,nav_arm_event_response_age_sec,"
      << "nav_arm_event_response_success,nav_arm_event_response_message";

    for (int i = 0; i < motor_count_; ++i) {
      csv_file_ << ",state_valid_" << i
                << ",state_q_rad_" << i
                << ",state_q_deg_" << i
                << ",state_dq_" << i
                << ",state_tau_est_" << i;
    }

    for (int i = 0; i < motor_count_; ++i) {
      csv_file_ << ",cmd_q_rad_" << i
                << ",cmd_q_deg_" << i
                << ",cmd_dq_" << i
                << ",cmd_tau_" << i
                << ",cmd_kp_" << i
                << ",cmd_kd_" << i;
    }

    csv_file_ << "\n";
    csv_file_.flush();
  }

  void write_csv_row(
    const rclcpp::Time & now_time,
    const robot_msgs::msg::RobotState & state,
    const robot_msgs::msg::RobotCommand & command,
    bool has_state,
    bool has_command,
    bool has_ready,
    bool driver_ready,
    double state_age,
    double command_age,
    double ready_age,
    const nav_msgs::msg::Odometry & nav_state,
    bool has_nav_state,
    double nav_state_age,
    const navigation::msg::MapPointArray & nav_task_points,
    bool has_nav_task_points,
    double nav_task_points_age,
    const std::pair<int, int> & nav_task_points_range,
    const navigation::srv::MissionCommand::Request & nav_mission_request,
    bool has_nav_mission_request,
    double nav_mission_request_age,
    const navigation::srv::MissionCommand::Response & nav_mission_response,
    bool has_nav_mission_response,
    double nav_mission_response_age,
    const navigation::srv::StringCommand::Request & nav_arm_event_request,
    bool has_nav_arm_event_request,
    double nav_arm_event_request_age,
    const navigation::srv::StringCommand::Response & nav_arm_event_response,
    bool has_nav_arm_event_response,
    double nav_arm_event_response_age,
    const ServiceState & service_state,
    bool action_ready)
  {
    if (!csv_file_.is_open()) {
      return;
    }

    const double elapsed = (now_time - start_ros_time_).seconds();
    const auto & nav_pose = nav_state.pose.pose;
    const double nav_yaw = has_nav_state ?
      yaw_from_quaternion_wxyz(
      nav_pose.orientation.w,
      nav_pose.orientation.x,
      nav_pose.orientation.y,
      nav_pose.orientation.z) : 0.0;

    csv_file_ << wall_timestamp_now()
              << "," << std::fixed << std::setprecision(9) << now_time.seconds()
              << "," << elapsed
              << "," << bool_text(has_ready)
              << "," << bool_text(driver_ready)
              << "," << ready_age
              << "," << bool_text(has_state)
              << "," << state_age
              << "," << state.motor_state.size()
              << "," << bool_text(has_command)
              << "," << command_age
              << "," << command.motor_command.size()
              << "," << bool_text(service_state.mode)
              << "," << bool_text(service_state.suction)
              << "," << bool_text(service_state.pick)
              << "," << bool_text(service_state.place)
              << "," << bool_text(service_state.stack)
              << "," << bool_text(service_state.payload_estimate)
              << "," << bool_text(service_state.payload_state)
              << "," << bool_text(service_state.arm_mission)
              << "," << bool_text(service_state.nav_arm_event)
              << "," << bool_text(action_ready)
              << "," << bool_text(has_nav_state)
              << "," << nav_state_age
              << "," << (has_nav_state ? nav_pose.position.x : 0.0)
              << "," << (has_nav_state ? nav_pose.position.y : 0.0)
              << "," << nav_yaw
              << "," << bool_text(has_nav_task_points)
              << "," << nav_task_points_age
              << "," << nav_task_points.points.size()
              << ","
              << (has_nav_task_points && !nav_task_points.points.empty() ? nav_task_points_range.first : 0)
              << ","
              << (has_nav_task_points && !nav_task_points.points.empty() ? nav_task_points_range.second : 0)
              << "," << format_task_points_csv(nav_task_points, has_nav_task_points)
              << "," << bool_text(has_nav_mission_request)
              << "," << nav_mission_request_age
              << "," << csv_escape(has_nav_mission_request ? nav_mission_request.action : "")
              << "," << (has_nav_mission_request ? nav_mission_request.task_index : 0)
              << "," << (has_nav_mission_request ? nav_mission_request.point_id : 0)
              << "," << (has_nav_mission_request ? nav_mission_request.x : 0.0)
              << "," << (has_nav_mission_request ? nav_mission_request.y : 0.0)
              << "," << bool_text(has_nav_mission_response)
              << "," << nav_mission_response_age
              << "," << bool_text(has_nav_mission_response && nav_mission_response.success)
              << ","
              << csv_escape(has_nav_mission_response ? nav_mission_response.message : "")
              << "," << bool_text(has_nav_arm_event_request)
              << "," << nav_arm_event_request_age
              << ","
              << csv_escape(has_nav_arm_event_request ? nav_arm_event_request.message : "")
              << "," << bool_text(has_nav_arm_event_response)
              << "," << nav_arm_event_response_age
              << "," << bool_text(has_nav_arm_event_response && nav_arm_event_response.success)
              << ","
              << csv_escape(has_nav_arm_event_response ? nav_arm_event_response.message : "");

    for (int i = 0; i < motor_count_; ++i) {
      if (has_state && i < static_cast<int>(state.motor_state.size())) {
        const auto & motor = state.motor_state[i];
        csv_file_ << "," << bool_text(motor.valid)
                  << "," << motor.q
                  << "," << motor.q * kRadToDeg
                  << "," << motor.dq
                  << "," << motor.tau_est;
      } else {
        csv_file_ << ",,,,,";
      }
    }

    for (int i = 0; i < motor_count_; ++i) {
      if (has_command && i < static_cast<int>(command.motor_command.size())) {
        const auto & motor = command.motor_command[i];
        csv_file_ << "," << motor.q
                  << "," << motor.q * kRadToDeg
                  << "," << motor.dq
                  << "," << motor.tau
                  << "," << motor.kp
                  << "," << motor.kd;
      } else {
        csv_file_ << ",,,,,,";
      }
    }

    csv_file_ << "\n";
    csv_file_.flush();
  }

  std::string state_topic_;
  std::string command_topic_;
  std::string ready_topic_;
  std::string nav_state_topic_;
  std::string nav_task_points_topic_;
  std::string arm_mission_service_;
  std::string nav_arm_event_service_;
  std::string nav_mission_request_topic_;
  std::string nav_mission_response_topic_;
  std::string nav_arm_event_request_topic_;
  std::string nav_arm_event_response_topic_;
  std::string csv_dir_;
  int motor_count_{5};
  bool csv_enabled_{true};
  std::chrono::system_clock::time_point start_wall_time_;
  rclcpp::Time start_ros_time_{0, 0, RCL_ROS_TIME};
  std::ofstream csv_file_;
  std::string csv_path_;

  mutable std::mutex mutex_;
  robot_msgs::msg::RobotState last_state_;
  robot_msgs::msg::RobotCommand last_command_;
  nav_msgs::msg::Odometry last_nav_state_;
  navigation::msg::MapPointArray last_nav_task_points_;
  navigation::srv::MissionCommand::Request last_nav_mission_request_;
  navigation::srv::MissionCommand::Response last_nav_mission_response_;
  navigation::srv::StringCommand::Request last_nav_arm_event_request_;
  navigation::srv::StringCommand::Response last_nav_arm_event_response_;
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ready_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_task_points_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_mission_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_mission_response_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_arm_event_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_arm_event_response_time_{0, 0, RCL_ROS_TIME};
  bool has_state_{false};
  bool has_command_{false};
  bool has_ready_{false};
  bool has_nav_state_{false};
  bool has_nav_task_points_{false};
  bool has_nav_mission_request_{false};
  bool has_nav_mission_response_{false};
  bool has_nav_arm_event_request_{false};
  bool has_nav_arm_event_response_{false};
  bool driver_ready_{false};

  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_state_sub_;
  rclcpp::Subscription<navigation::msg::MapPointArray>::SharedPtr nav_task_points_sub_;
  rclcpp::Subscription<navigation::srv::MissionCommand::Request>::SharedPtr nav_mission_request_sub_;
  rclcpp::Subscription<navigation::srv::MissionCommand::Response>::SharedPtr nav_mission_response_sub_;
  rclcpp::Subscription<navigation::srv::StringCommand::Request>::SharedPtr nav_arm_event_request_sub_;
  rclcpp::Subscription<navigation::srv::StringCommand::Response>::SharedPtr nav_arm_event_response_sub_;
  rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
  rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client_;
  rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr place_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr stack_client_;
  rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_estimate_client_;
  rclcpp::Client<robot_msgs::srv::SetPayloadState>::SharedPtr payload_state_client_;
  rclcpp::Client<navigation::srv::MissionCommand>::SharedPtr arm_mission_client_;
  rclcpp::Client<navigation::srv::StringCommand>::SharedPtr nav_arm_event_client_;
  rclcpp_action::Client<MoveJoint>::SharedPtr move_joint_client_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugToolNode>());
  rclcpp::shutdown();
  return 0;
}
