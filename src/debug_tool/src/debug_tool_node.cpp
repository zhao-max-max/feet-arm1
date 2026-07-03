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
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = *msg;
        last_state_time_ = now();
        has_state_ = true;
      });

    command_sub_ = create_subscription<robot_msgs::msg::RobotCommand>(
      command_topic_, state_qos,
      [this](const robot_msgs::msg::RobotCommand::SharedPtr msg) {
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

    mode_client_ = create_client<robot_msgs::srv::SetControllerMode>("set_controller_mode");
    suction_client_ = create_client<robot_msgs::srv::SetSuction>("set_suction");
    pick_client_ = create_client<robot_msgs::srv::GetPickPos>("get_pick_pos");
    place_client_ = create_client<robot_msgs::srv::GetPlacePos>("get_place_pos");
    stack_client_ = create_client<robot_msgs::srv::GetPlacePos>("get_stack_pos");
    payload_estimate_client_ =
      create_client<robot_msgs::srv::GetPayloadEstimate>("get_payload_estimate");
    payload_state_client_ =
      create_client<robot_msgs::srv::SetPayloadState>("set_payload_state");
    move_joint_client_ = rclcpp_action::create_client<MoveJoint>(this, "move_joint");

    report_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(report_period_sec)),
      std::bind(&DebugToolNode::report, this));

    RCLCPP_INFO(
      get_logger(),
      "debug_tool_node started. state_topic=%s command_topic=%s ready_topic=%s report_period=%.2fs csv=%s",
      state_topic_.c_str(), command_topic_.c_str(), ready_topic_.c_str(), report_period_sec,
      csv_path_.empty() ? "disabled" : csv_path_.c_str());
  }

private:
  void report()
  {
    robot_msgs::msg::RobotState state_snapshot;
    robot_msgs::msg::RobotCommand command_snapshot;
    rclcpp::Time state_time;
    rclcpp::Time command_time;
    rclcpp::Time ready_time;
    bool has_state = false;
    bool has_command = false;
    bool has_ready = false;
    bool driver_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_snapshot = last_state_;
      command_snapshot = last_command_;
      state_time = last_state_time_;
      command_time = last_command_time_;
      ready_time = last_ready_time_;
      has_state = has_state_;
      has_command = has_command_;
      has_ready = has_ready_;
      driver_ready = driver_ready_;
    }

    const auto now_time = now();
    const double state_age = has_state ? (now_time - state_time).seconds() : -1.0;
    const double command_age = has_command ? (now_time - command_time).seconds() : -1.0;
    const double ready_age = has_ready ? (now_time - ready_time).seconds() : -1.0;
    const auto service_state = read_service_state();
    const bool action_ready = move_joint_client_->action_server_is_ready();

    write_csv_row(
      now_time, state_snapshot, command_snapshot, has_state, has_command, has_ready,
      driver_ready, state_age, command_age, ready_age, service_state, action_ready);

    if (!has_state) {
      RCLCPP_WARN(
        get_logger(),
        "ready=%s ready_age=%.2fs cmd_age=%.2fs | no robot state received yet | services=%s action=%s",
        driver_ready ? "true" : "false", ready_age, command_age,
        service_summary(service_state).c_str(), ready_word(action_ready));
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "ready=%s ready_age=%.2fs state_age=%.2fs cmd_age=%.2fs valid=%s q_deg=[%s] dq=[%s] cmd_q_deg=[%s] | services=%s action=%s",
      driver_ready ? "true" : "false",
      ready_age,
      state_age,
      command_age,
      format_valid(state_snapshot).c_str(),
      format_positions_deg(state_snapshot).c_str(),
      format_velocities(state_snapshot).c_str(),
      has_command ? format_command_positions_deg(command_snapshot).c_str() : "",
      service_summary(service_state).c_str(),
      ready_word(action_ready));
  }

  struct ServiceState
  {
    bool mode{false};
    bool suction{false};
    bool pick{false};
    bool place{false};
    bool stack{false};
    bool payload_estimate{false};
    bool payload_state{false};
  };

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
        << " payload_state:" << ready_word(state.payload_state);
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
      << "service_payload_estimate,service_payload_state,action_move_joint";

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
    const ServiceState & service_state,
    bool action_ready)
  {
    if (!csv_file_.is_open()) {
      return;
    }

    const double elapsed = (now_time - start_ros_time_).seconds();
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
              << "," << bool_text(action_ready);

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
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ready_time_{0, 0, RCL_ROS_TIME};
  bool has_state_{false};
  bool has_command_{false};
  bool has_ready_{false};
  bool driver_ready_{false};

  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
  rclcpp::Client<robot_msgs::srv::SetSuction>::SharedPtr suction_client_;
  rclcpp::Client<robot_msgs::srv::GetPickPos>::SharedPtr pick_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr place_client_;
  rclcpp::Client<robot_msgs::srv::GetPlacePos>::SharedPtr stack_client_;
  rclcpp::Client<robot_msgs::srv::GetPayloadEstimate>::SharedPtr payload_estimate_client_;
  rclcpp::Client<robot_msgs::srv::SetPayloadState>::SharedPtr payload_state_client_;
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
