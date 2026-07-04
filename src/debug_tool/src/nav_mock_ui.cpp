#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <QAbstractItemView>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSplitter>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigation/msg/map_point.hpp"
#include "navigation/msg/map_point_array.hpp"
#include "navigation/srv/mission_command.hpp"
#include "navigation/srv/string_command.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr int kMaxLogLines = 200;

struct TaskPointState
{
  int id{0};
  double x{0.0};
  double y{0.0};
  std::uint8_t task_type{navigation::msg::MapPoint::TASK_NONE};
  std::string event_label;
};

std::string wall_time_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&raw, &local_time);

  std::ostringstream out;
  out << std::put_time(&local_time, "%H:%M:%S");
  return out.str();
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

double yaw_from_quaternion_wxyz(double w, double x, double y, double z)
{
  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z));
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string format_xy(double x, double y)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << "(" << x << ", " << y << ")";
  return out.str();
}

std::string format_pose(double x, double y, double yaw_rad)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "x=" << x
      << " y=" << y
      << " yaw=" << yaw_rad << "rad (" << yaw_rad * kRadToDeg << "deg)";
  return out.str();
}

std::string format_mission_request(const navigation::srv::MissionCommand::Request & request)
{
  std::ostringstream out;
  out << request.action
      << " idx=" << request.task_index
      << " point=" << request.point_id
      << " map=" << format_xy(request.x, request.y);
  return out.str();
}

std::string format_mission_response(const navigation::srv::MissionCommand::Response & response)
{
  std::ostringstream out;
  out << (response.success ? "ok" : "fail") << ": " << response.message;
  return out.str();
}

std::string format_string_command_response(const navigation::srv::StringCommand::Response & response)
{
  std::ostringstream out;
  out << (response.success ? "ok" : "fail") << ": " << response.message;
  return out.str();
}

double clamp_hz(double hz)
{
  return std::max(0.1, std::min(100.0, hz));
}
}  // namespace

class NavMockRosNode : public rclcpp::Node
{
public:
  struct Snapshot
  {
    bool pose_auto_publish{true};
    double pose_publish_hz{10.0};
    double pose_x{0.0};
    double pose_y{0.0};
    double pose_yaw_rad{0.0};
    std::string state_frame_id{"map"};
    std::string state_child_frame_id{"base_link"};
    std::vector<TaskPointState> task_points;
    bool mission_service_ready{false};
    bool arm_event_response_success{true};
    std::string arm_event_response_message{"ack"};
    std::string last_mission_call_status{"idle"};
    bool has_last_arm_event_request{false};
    std::string last_arm_event_request;
    double last_arm_event_request_age_sec{-1.0};
    bool has_debug_mission_request{false};
    std::string debug_mission_request;
    double debug_mission_request_age_sec{-1.0};
    bool has_debug_mission_response{false};
    std::string debug_mission_response;
    double debug_mission_response_age_sec{-1.0};
    bool has_debug_arm_event_request{false};
    std::string debug_arm_event_request;
    double debug_arm_event_request_age_sec{-1.0};
    bool has_debug_arm_event_response{false};
    std::string debug_arm_event_response;
    double debug_arm_event_response_age_sec{-1.0};
    bool has_target_pose{false};
    geometry_msgs::msg::Pose target_pose;
    double target_pose_age_sec{-1.0};
    std::vector<std::string> logs;
  };

  NavMockRosNode()
  : Node("nav_mock_ui_node")
  {
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
    target_pose_topic_ = declare_parameter<std::string>("target_pose_topic", "/task/target_pose");
    state_frame_id_ = declare_parameter<std::string>("state_frame_id", "map");
    state_child_frame_id_ = declare_parameter<std::string>("state_child_frame_id", "base_link");
    task_points_frame_id_ = declare_parameter<std::string>("task_points_frame_id", "map");
    pose_auto_publish_ = declare_parameter<bool>("pose_auto_publish", true);
    pose_publish_hz_ = clamp_hz(declare_parameter<double>("pose_publish_hz", 10.0));
    pose_x_ = declare_parameter<double>("initial_pose_x", 0.0);
    pose_y_ = declare_parameter<double>("initial_pose_y", 0.0);
    pose_yaw_rad_ = declare_parameter<double>("initial_pose_yaw_rad", 0.0);
    arm_event_response_success_ =
      declare_parameter<bool>("arm_event_response_success", true);
    arm_event_response_message_ =
      declare_parameter<std::string>("arm_event_response_message", "ack");

    default_task_points_ = load_default_task_points_from_parameters();
    task_points_ = default_task_points_;

    nav_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      nav_state_topic_, rclcpp::SensorDataQoS());
    auto task_points_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    nav_task_points_pub_ = create_publisher<navigation::msg::MapPointArray>(
      nav_task_points_topic_, task_points_qos);

    arm_mission_client_ =
      create_client<navigation::srv::MissionCommand>(arm_mission_service_);
    nav_arm_event_server_ = create_service<navigation::srv::StringCommand>(
      nav_arm_event_service_,
      [this](
        const navigation::srv::StringCommand::Request::SharedPtr request,
        navigation::srv::StringCommand::Response::SharedPtr response)
      {
        std::string request_text = request ? request->message : "<null>";
        bool success = true;
        std::string message;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          has_last_arm_event_request_ = true;
          last_arm_event_request_ = request_text;
          last_arm_event_request_time_ = now();
          success = arm_event_response_success_;
          message = arm_event_response_message_;
        }

        response->success = success;
        response->message = message;
        append_log(
          "arm_event request: " + request_text +
          " -> response " + format_string_command_response(*response));
      });

    auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    mission_request_sub_ =
      create_subscription<navigation::srv::MissionCommand::Request>(
      nav_mission_request_topic_, debug_qos,
      [this](const navigation::srv::MissionCommand::Request::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        has_debug_mission_request_ = true;
        last_debug_mission_request_ = format_mission_request(*msg);
        last_debug_mission_request_time_ = now();
      });

    mission_response_sub_ =
      create_subscription<navigation::srv::MissionCommand::Response>(
      nav_mission_response_topic_, debug_qos,
      [this](const navigation::srv::MissionCommand::Response::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        has_debug_mission_response_ = true;
        last_debug_mission_response_ = format_mission_response(*msg);
        last_debug_mission_response_time_ = now();
      });

    arm_event_request_sub_ =
      create_subscription<navigation::srv::StringCommand::Request>(
      nav_arm_event_request_topic_, debug_qos,
      [this](const navigation::srv::StringCommand::Request::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        has_debug_arm_event_request_ = true;
        last_debug_arm_event_request_ = msg->message;
        last_debug_arm_event_request_time_ = now();
      });

    arm_event_response_sub_ =
      create_subscription<navigation::srv::StringCommand::Response>(
      nav_arm_event_response_topic_, debug_qos,
      [this](const navigation::srv::StringCommand::Response::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        has_debug_arm_event_response_ = true;
        last_debug_arm_event_response_ = format_string_command_response(*msg);
        last_debug_arm_event_response_time_ = now();
      });

    target_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      target_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_target_pose_ = *msg;
        has_target_pose_ = true;
        last_target_pose_time_ = now();
      });

    pose_timer_ = create_wall_timer(20ms, std::bind(&NavMockRosNode::maybe_publish_pose, this));

    publish_task_points();
    publish_pose_once();
    append_log(
      "nav mock UI ready: state_topic=" + nav_state_topic_ +
      " task_points_topic=" + nav_task_points_topic_ +
      " arm_mission_service=" + arm_mission_service_ +
      " arm_event_service=" + nav_arm_event_service_);
  }

  Snapshot snapshot() const
  {
    Snapshot out;
    const auto t_now = now();
    std::lock_guard<std::mutex> lock(mutex_);
    out.pose_auto_publish = pose_auto_publish_;
    out.pose_publish_hz = pose_publish_hz_;
    out.pose_x = pose_x_;
    out.pose_y = pose_y_;
    out.pose_yaw_rad = pose_yaw_rad_;
    out.state_frame_id = state_frame_id_;
    out.state_child_frame_id = state_child_frame_id_;
    out.task_points = task_points_;
    out.arm_event_response_success = arm_event_response_success_;
    out.arm_event_response_message = arm_event_response_message_;
    out.last_mission_call_status = last_mission_call_status_;
    out.has_last_arm_event_request = has_last_arm_event_request_;
    out.last_arm_event_request = last_arm_event_request_;
    out.last_arm_event_request_age_sec =
      has_last_arm_event_request_ ? (t_now - last_arm_event_request_time_).seconds() : -1.0;
    out.has_debug_mission_request = has_debug_mission_request_;
    out.debug_mission_request = last_debug_mission_request_;
    out.debug_mission_request_age_sec =
      has_debug_mission_request_ ? (t_now - last_debug_mission_request_time_).seconds() : -1.0;
    out.has_debug_mission_response = has_debug_mission_response_;
    out.debug_mission_response = last_debug_mission_response_;
    out.debug_mission_response_age_sec =
      has_debug_mission_response_ ? (t_now - last_debug_mission_response_time_).seconds() : -1.0;
    out.has_debug_arm_event_request = has_debug_arm_event_request_;
    out.debug_arm_event_request = last_debug_arm_event_request_;
    out.debug_arm_event_request_age_sec =
      has_debug_arm_event_request_ ? (t_now - last_debug_arm_event_request_time_).seconds() : -1.0;
    out.has_debug_arm_event_response = has_debug_arm_event_response_;
    out.debug_arm_event_response = last_debug_arm_event_response_;
    out.debug_arm_event_response_age_sec =
      has_debug_arm_event_response_ ? (t_now - last_debug_arm_event_response_time_).seconds() : -1.0;
    out.has_target_pose = has_target_pose_;
    out.target_pose = last_target_pose_;
    out.target_pose_age_sec =
      has_target_pose_ ? (t_now - last_target_pose_time_).seconds() : -1.0;
    out.logs.assign(log_lines_.begin(), log_lines_.end());
    out.mission_service_ready = arm_mission_client_->service_is_ready();
    return out;
  }

  std::vector<TaskPointState> default_task_points() const
  {
    return default_task_points_;
  }

  void set_pose(double x, double y, double yaw_rad)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_x_ = x;
    pose_y_ = y;
    pose_yaw_rad_ = yaw_rad;
  }

  void set_pose_publish_enabled(bool enabled)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose_auto_publish_ = enabled;
    }
    append_log(std::string("pose auto publish ") + (enabled ? "enabled" : "disabled"));
  }

  void set_pose_publish_hz(double hz)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose_publish_hz_ = clamp_hz(hz);
    }
    append_log("pose publish rate set to " + format_double(clamp_hz(hz), 1) + " Hz");
  }

  void set_arm_event_response(bool success, const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_event_response_success_ = success;
    arm_event_response_message_ = message;
  }

  void publish_pose_once()
  {
    nav_msgs::msg::Odometry msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.header.stamp = now();
      msg.header.frame_id = state_frame_id_;
      msg.child_frame_id = state_child_frame_id_;
      msg.pose.pose.position.x = pose_x_;
      msg.pose.pose.position.y = pose_y_;
      msg.pose.pose.position.z = 0.0;
      msg.pose.pose.orientation = quaternion_from_yaw(pose_yaw_rad_);
    }
    nav_state_pub_->publish(msg);
  }

  void set_task_points(const std::vector<TaskPointState> & points)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      task_points_ = points;
    }
    publish_task_points();
  }

  void publish_task_points()
  {
    navigation::msg::MapPointArray msg;
    std::vector<TaskPointState> points_copy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      points_copy = task_points_;
      msg.header.stamp = now();
      msg.header.frame_id = task_points_frame_id_;
    }

    msg.points.reserve(points_copy.size());
    for (const auto & point : points_copy) {
      navigation::msg::MapPoint out;
      out.id = point.id;
      out.x = point.x;
      out.y = point.y;
      out.fast = false;
      out.constant_speed = false;
      out.segment_custom_speed = false;
      out.segment_constant_speed = false;
      out.segment_speed_level = 0;
      out.segment_linear_x = 0.0;
      out.segment_max_angular_speed = 0.0;
      out.segment_k_alpha = 0.0;
      out.segment_k_beta = 0.0;
      out.task_type = point.task_type;
      out.event_label = point.event_label;
      msg.points.push_back(out);
    }

    nav_task_points_pub_->publish(msg);
  }

  void send_mission(
    const std::string & action,
    std::uint32_t task_index,
    int point_id,
    double x,
    double y)
  {
    if (!arm_mission_client_->service_is_ready()) {
      append_log("mission service not ready: " + arm_mission_service_);
      std::lock_guard<std::mutex> lock(mutex_);
      last_mission_call_status_ = "service not ready";
      return;
    }

    auto request = std::make_shared<navigation::srv::MissionCommand::Request>();
    request->action = action;
    request->task_index = task_index;
    request->point_id = point_id;
    request->x = x;
    request->y = y;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_mission_call_status_ = "requesting " + format_mission_request(*request);
    }
    append_log("mission request -> " + format_mission_request(*request));

    arm_mission_client_->async_send_request(
      request,
      [this, action](rclcpp::Client<navigation::srv::MissionCommand>::SharedFuture future) {
        const auto response = future.get();
        const std::string text = response ?
          format_mission_response(*response) : std::string("null response");
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_mission_call_status_ = action + " => " + text;
        }
        append_log("mission response <- " + text);
      });
  }

private:
  std::vector<TaskPointState> load_default_task_points_from_parameters()
  {
    const auto ids = declare_parameter<std::vector<int64_t>>(
      "default_task_point_ids",
      std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
    const auto xs = declare_parameter<std::vector<double>>(
      "default_task_point_x",
      std::vector<double>{
        -1.251139, -0.428469, 0.391736, 1.223170,
        1.227952, 0.405132, -0.439627, -1.260851,
        -1.184853, -0.390241, 0.375382, 1.181787});
    const auto ys = declare_parameter<std::vector<double>>(
      "default_task_point_y",
      std::vector<double>{
        2.427514, 2.425345, 2.424333, 2.419070,
        1.589189, 1.587631, 1.587582, 1.589028,
        4.910509, 4.901606, 4.902796, 4.901133});

    const size_t count = std::min(ids.size(), std::min(xs.size(), ys.size()));
    std::vector<TaskPointState> points;
    points.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      TaskPointState point;
      point.id = static_cast<int>(ids[i]);
      point.x = xs[i];
      point.y = ys[i];
      points.push_back(point);
    }
    return points;
  }

  static std::string format_double(double value, int decimals)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
  }

  void maybe_publish_pose()
  {
    bool should_publish = false;
    double publish_hz = 10.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      should_publish = pose_auto_publish_;
      publish_hz = pose_publish_hz_;
    }

    if (!should_publish) {
      return;
    }

    const auto now_wall = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / clamp_hz(publish_hz));
    if (last_pose_publish_wall_.time_since_epoch().count() == 0 ||
        (now_wall - last_pose_publish_wall_) >= period)
    {
      publish_pose_once();
      last_pose_publish_wall_ = now_wall;
    }
  }

  void append_log(const std::string & text)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    log_lines_.push_back("[" + wall_time_now() + "] " + text);
    while (static_cast<int>(log_lines_.size()) > kMaxLogLines) {
      log_lines_.pop_front();
    }
  }

  mutable std::mutex mutex_;
  std::string nav_state_topic_;
  std::string nav_task_points_topic_;
  std::string arm_mission_service_;
  std::string nav_arm_event_service_;
  std::string nav_mission_request_topic_;
  std::string nav_mission_response_topic_;
  std::string nav_arm_event_request_topic_;
  std::string nav_arm_event_response_topic_;
  std::string target_pose_topic_;
  std::string state_frame_id_;
  std::string state_child_frame_id_;
  std::string task_points_frame_id_;
  bool pose_auto_publish_{true};
  double pose_publish_hz_{10.0};
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_rad_{0.0};
  bool arm_event_response_success_{true};
  std::string arm_event_response_message_{"ack"};
  std::vector<TaskPointState> default_task_points_;
  std::vector<TaskPointState> task_points_;
  std::string last_mission_call_status_{"idle"};
  bool has_last_arm_event_request_{false};
  std::string last_arm_event_request_;
  rclcpp::Time last_arm_event_request_time_{0, 0, RCL_ROS_TIME};
  bool has_debug_mission_request_{false};
  std::string last_debug_mission_request_;
  rclcpp::Time last_debug_mission_request_time_{0, 0, RCL_ROS_TIME};
  bool has_debug_mission_response_{false};
  std::string last_debug_mission_response_;
  rclcpp::Time last_debug_mission_response_time_{0, 0, RCL_ROS_TIME};
  bool has_debug_arm_event_request_{false};
  std::string last_debug_arm_event_request_;
  rclcpp::Time last_debug_arm_event_request_time_{0, 0, RCL_ROS_TIME};
  bool has_debug_arm_event_response_{false};
  std::string last_debug_arm_event_response_;
  rclcpp::Time last_debug_arm_event_response_time_{0, 0, RCL_ROS_TIME};
  bool has_target_pose_{false};
  geometry_msgs::msg::Pose last_target_pose_;
  rclcpp::Time last_target_pose_time_{0, 0, RCL_ROS_TIME};
  std::deque<std::string> log_lines_;
  std::chrono::steady_clock::time_point last_pose_publish_wall_{};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav_state_pub_;
  rclcpp::Publisher<navigation::msg::MapPointArray>::SharedPtr nav_task_points_pub_;
  rclcpp::Client<navigation::srv::MissionCommand>::SharedPtr arm_mission_client_;
  rclcpp::Service<navigation::srv::StringCommand>::SharedPtr nav_arm_event_server_;
  rclcpp::Subscription<navigation::srv::MissionCommand::Request>::SharedPtr mission_request_sub_;
  rclcpp::Subscription<navigation::srv::MissionCommand::Response>::SharedPtr mission_response_sub_;
  rclcpp::Subscription<navigation::srv::StringCommand::Request>::SharedPtr arm_event_request_sub_;
  rclcpp::Subscription<navigation::srv::StringCommand::Response>::SharedPtr arm_event_response_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
};

class NavMapView : public QWidget
{
public:
  struct DrawState
  {
    std::vector<TaskPointState> task_points;
    double pose_x{0.0};
    double pose_y{0.0};
    double pose_yaw_rad{0.0};
    bool has_target_pose{false};
    geometry_msgs::msg::Pose target_pose;
    int selected_task_point_id{0};
    bool has_selection{false};
  };

  explicit NavMapView(QWidget * parent = nullptr)
  : QWidget(parent)
  {
    setMinimumSize(520, 520);
    setMouseTracking(true);
  }

  void set_draw_state(const DrawState & state)
  {
    state_ = state;
    update();
  }

  void set_task_point_selected_callback(std::function<void(int)> callback)
  {
    on_task_point_selected_ = std::move(callback);
  }

  void set_pose_set_callback(std::function<void(double, double)> callback)
  {
    on_pose_set_ = std::move(callback);
  }

protected:
  void paintEvent(QPaintEvent * event) override
  {
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.fillRect(rect(), QColor(245, 247, 250));
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF canvas = rect().adjusted(16, 16, -16, -16);
    draw_grid(&painter, canvas);
    draw_points(&painter, canvas);
    draw_target_pose(&painter, canvas);
    draw_pose(&painter, canvas);
    draw_hint(&painter, canvas);
  }

  void mousePressEvent(QMouseEvent * event) override
  {
    if (!event) {
      return;
    }

    const QRectF canvas = rect().adjusted(16, 16, -16, -16);
    if (!canvas.contains(event->localPos())) {
      return;
    }

    if (event->button() == Qt::LeftButton) {
      const auto world = screen_to_world(event->localPos(), canvas);
      const int nearest_id = nearest_task_point_id(world.first, world.second, canvas);
      if (nearest_id != std::numeric_limits<int>::min() && on_task_point_selected_) {
        on_task_point_selected_(nearest_id);
      }
    } else if (event->button() == Qt::RightButton) {
      const auto world = screen_to_world(event->localPos(), canvas);
      if (on_pose_set_) {
        on_pose_set_(world.first, world.second);
      }
    }
  }

private:
  void draw_grid(QPainter * painter, const QRectF & canvas)
  {
    painter->save();
    painter->setPen(QPen(QColor(220, 224, 230), 1.0));
    for (int i = 0; i <= 10; ++i) {
      const double x = canvas.left() + canvas.width() * static_cast<double>(i) / 10.0;
      const double y = canvas.top() + canvas.height() * static_cast<double>(i) / 10.0;
      painter->drawLine(QPointF(x, canvas.top()), QPointF(x, canvas.bottom()));
      painter->drawLine(QPointF(canvas.left(), y), QPointF(canvas.right(), y));
    }

    painter->setPen(QPen(QColor(180, 185, 192), 1.5));
    const auto origin = world_to_screen(0.0, 0.0, canvas);
    painter->drawLine(QPointF(origin.first, canvas.top()), QPointF(origin.first, canvas.bottom()));
    painter->drawLine(QPointF(canvas.left(), origin.second), QPointF(canvas.right(), origin.second));
    painter->restore();
  }

  void draw_points(QPainter * painter, const QRectF & canvas)
  {
    for (const auto & point : state_.task_points) {
      const auto screen = world_to_screen(point.x, point.y, canvas);
      const bool selected = state_.has_selection && point.id == state_.selected_task_point_id;
      painter->setPen(QPen(selected ? QColor(178, 34, 34) : QColor(18, 96, 160), selected ? 2.5 : 1.5));
      painter->setBrush(selected ? QColor(230, 87, 87) : QColor(68, 149, 227));
      painter->drawEllipse(QPointF(screen.first, screen.second), selected ? 7.0 : 5.0, selected ? 7.0 : 5.0);

      painter->setPen(QColor(35, 42, 52));
      painter->drawText(
        QRectF(screen.first + 8.0, screen.second - 16.0, 80.0, 20.0),
        QStringLiteral("#%1").arg(point.id));
    }
  }

  void draw_pose(QPainter * painter, const QRectF & canvas)
  {
    const auto screen = world_to_screen(state_.pose_x, state_.pose_y, canvas);
    painter->save();
    painter->setPen(QPen(QColor(34, 139, 34), 2.0));
    painter->setBrush(QColor(46, 204, 113));
    painter->drawEllipse(QPointF(screen.first, screen.second), 7.0, 7.0);

    const double arrow_len = 28.0;
    const QPointF tip(
      screen.first + std::cos(state_.pose_yaw_rad) * arrow_len,
      screen.second - std::sin(state_.pose_yaw_rad) * arrow_len);
    painter->drawLine(QPointF(screen.first, screen.second), tip);
    painter->restore();
  }

  void draw_target_pose(QPainter * painter, const QRectF & canvas)
  {
    if (!state_.has_target_pose) {
      return;
    }

    const auto screen = world_to_screen(
      state_.target_pose.position.x,
      state_.target_pose.position.y,
      canvas);
    painter->save();
    painter->setPen(QPen(QColor(123, 63, 0), 2.0, Qt::DashLine));
    painter->drawLine(
      QPointF(screen.first - 10.0, screen.second),
      QPointF(screen.first + 10.0, screen.second));
    painter->drawLine(
      QPointF(screen.first, screen.second - 10.0),
      QPointF(screen.first, screen.second + 10.0));
    painter->drawText(
      QRectF(screen.first + 8.0, screen.second + 8.0, 120.0, 20.0),
      QStringLiteral("target_pose"));
    painter->restore();
  }

  void draw_hint(QPainter * painter, const QRectF & canvas)
  {
    painter->save();
    painter->setPen(QColor(90, 99, 109));
    painter->drawText(
      QRectF(canvas.left(), canvas.bottom() + 4.0, canvas.width(), 20.0),
      Qt::AlignLeft | Qt::AlignVCenter,
      QStringLiteral("Left click: select task point   Right click: set lidar pose x/y"));
    painter->restore();
  }

  std::pair<double, double> screen_to_world(const QPointF & screen, const QRectF & canvas) const
  {
    const auto bounds = world_bounds();
    const double x = bounds.min_x + (screen.x() - canvas.left()) / canvas.width() * (bounds.max_x - bounds.min_x);
    const double y = bounds.max_y - (screen.y() - canvas.top()) / canvas.height() * (bounds.max_y - bounds.min_y);
    return {x, y};
  }

  std::pair<double, double> world_to_screen(double x, double y, const QRectF & canvas) const
  {
    const auto bounds = world_bounds();
    const double sx = canvas.left() + (x - bounds.min_x) / (bounds.max_x - bounds.min_x) * canvas.width();
    const double sy = canvas.top() + (bounds.max_y - y) / (bounds.max_y - bounds.min_y) * canvas.height();
    return {sx, sy};
  }

  struct Bounds
  {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
  };

  Bounds world_bounds() const
  {
    double min_x = state_.pose_x;
    double max_x = state_.pose_x;
    double min_y = state_.pose_y;
    double max_y = state_.pose_y;

    for (const auto & point : state_.task_points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    if (state_.has_target_pose) {
      min_x = std::min(min_x, state_.target_pose.position.x);
      max_x = std::max(max_x, state_.target_pose.position.x);
      min_y = std::min(min_y, state_.target_pose.position.y);
      max_y = std::max(max_y, state_.target_pose.position.y);
    }

    if (std::abs(max_x - min_x) < 1e-6) {
      min_x -= 1.0;
      max_x += 1.0;
    }
    if (std::abs(max_y - min_y) < 1e-6) {
      min_y -= 1.0;
      max_y += 1.0;
    }

    const double padding_x = std::max(0.5, (max_x - min_x) * 0.15);
    const double padding_y = std::max(0.5, (max_y - min_y) * 0.15);
    return Bounds{
      min_x - padding_x,
      max_x + padding_x,
      min_y - padding_y,
      max_y + padding_y};
  }

  int nearest_task_point_id(double x, double y, const QRectF & canvas) const
  {
    const auto query_screen = world_to_screen(x, y, canvas);
    double best_px_distance = std::numeric_limits<double>::infinity();
    int best_id = std::numeric_limits<int>::min();
    for (const auto & point : state_.task_points) {
      const auto screen = world_to_screen(point.x, point.y, canvas);
      const double dx = screen.first - query_screen.first;
      const double dy = screen.second - query_screen.second;
      const double distance = std::hypot(dx, dy);
      if (distance < best_px_distance) {
        best_px_distance = distance;
        best_id = point.id;
      }
    }
    return best_px_distance <= 24.0 ? best_id : std::numeric_limits<int>::min();
  }

  DrawState state_;
  std::function<void(int)> on_task_point_selected_;
  std::function<void(double, double)> on_pose_set_;
};

class NavMockWindow : public QMainWindow
{
public:
  explicit NavMockWindow(std::shared_ptr<NavMockRosNode> node)
  : node_(std::move(node))
  {
    setWindowTitle(QStringLiteral("Navigation Mock Debug Tool"));
    resize(1560, 920);

    auto * central = new QWidget(this);
    auto * root_layout = new QVBoxLayout(central);

    status_label_ = new QLabel(QStringLiteral("waiting for ROS state"), central);
    status_label_->setMinimumHeight(28);
    root_layout->addWidget(status_label_);

    auto * splitter = new QSplitter(Qt::Horizontal, central);
    root_layout->addWidget(splitter, 1);

    auto * left_panel = new QWidget(splitter);
    auto * left_layout = new QVBoxLayout(left_panel);
    map_view_ = new NavMapView(left_panel);
    left_layout->addWidget(map_view_, 1);
    splitter->addWidget(left_panel);

    auto * right_panel = new QWidget(splitter);
    auto * right_layout = new QVBoxLayout(right_panel);
    right_layout->addWidget(build_pose_group(right_panel));
    right_layout->addWidget(build_mission_group(right_panel));
    right_layout->addWidget(build_arm_event_group(right_panel));
    right_layout->addWidget(build_debug_group(right_panel));
    right_layout->addWidget(build_task_points_group(right_panel), 1);
    right_layout->addWidget(build_logs_group(right_panel), 1);
    splitter->addWidget(right_panel);
    splitter->setStretchFactor(0, 2);
    splitter->setStretchFactor(1, 1);

    setCentralWidget(central);

    const auto initial_points = node_->default_task_points();
    load_task_points_into_table(initial_points);

    connect_signals();
    refresh_from_snapshot();

    refresh_timer_ = new QTimer(this);
    refresh_timer_->setInterval(120);
    connect(refresh_timer_, &QTimer::timeout, this, [this]() {
      refresh_from_snapshot();
    });
    refresh_timer_->start();
  }

private:
  QWidget * build_pose_group(QWidget * parent)
  {
    auto * group = new QGroupBox(QStringLiteral("Lidar Pose Publisher"), parent);
    auto * layout = new QGridLayout(group);

    pose_x_spin_ = make_double_spin(-100.0, 100.0, 3, 0.01, group);
    pose_y_spin_ = make_double_spin(-100.0, 100.0, 3, 0.01, group);
    pose_yaw_deg_spin_ = make_double_spin(-360.0, 360.0, 1, 1.0, group);
    pose_publish_hz_spin_ = make_double_spin(0.1, 100.0, 1, 0.5, group);
    auto_publish_check_ = new QCheckBox(QStringLiteral("Auto Publish"), group);
    publish_pose_button_ = new QPushButton(QStringLiteral("Publish Once"), group);
    pose_note_label_ =
      new QLabel(QStringLiteral("Publishing lidar pose in map frame. Right-click map to move x/y."), group);

    layout->addWidget(new QLabel(QStringLiteral("x (m)"), group), 0, 0);
    layout->addWidget(pose_x_spin_, 0, 1);
    layout->addWidget(new QLabel(QStringLiteral("y (m)"), group), 0, 2);
    layout->addWidget(pose_y_spin_, 0, 3);
    layout->addWidget(new QLabel(QStringLiteral("yaw (deg)"), group), 1, 0);
    layout->addWidget(pose_yaw_deg_spin_, 1, 1);
    layout->addWidget(new QLabel(QStringLiteral("rate (Hz)"), group), 1, 2);
    layout->addWidget(pose_publish_hz_spin_, 1, 3);
    layout->addWidget(auto_publish_check_, 2, 0, 1, 2);
    layout->addWidget(publish_pose_button_, 2, 2, 1, 2);
    layout->addWidget(pose_note_label_, 3, 0, 1, 4);
    return group;
  }

  QWidget * build_mission_group(QWidget * parent)
  {
    auto * group = new QGroupBox(QStringLiteral("Mission Command"), parent);
    auto * layout = new QGridLayout(group);

    action_combo_ = new QComboBox(group);
    action_combo_->addItem(QStringLiteral("ready"));
    action_combo_->addItem(QStringLiteral("pickup"));
    action_combo_->addItem(QStringLiteral("place"));
    task_index_spin_ = new QSpinBox(group);
    task_index_spin_->setRange(0, 1000000);
    point_id_spin_ = new QSpinBox(group);
    point_id_spin_->setRange(-1000000, 1000000);
    mission_x_spin_ = make_double_spin(-100.0, 100.0, 3, 0.01, group);
    mission_y_spin_ = make_double_spin(-100.0, 100.0, 3, 0.01, group);
    use_selected_button_ = new QPushButton(QStringLiteral("Fill From Selected Point"), group);
    send_mission_button_ = new QPushButton(QStringLiteral("Send Mission"), group);
    mission_status_label_ = new QLabel(QStringLiteral("idle"), group);

    layout->addWidget(new QLabel(QStringLiteral("action"), group), 0, 0);
    layout->addWidget(action_combo_, 0, 1);
    layout->addWidget(new QLabel(QStringLiteral("task_index"), group), 0, 2);
    layout->addWidget(task_index_spin_, 0, 3);
    layout->addWidget(new QLabel(QStringLiteral("point_id"), group), 1, 0);
    layout->addWidget(point_id_spin_, 1, 1);
    layout->addWidget(new QLabel(QStringLiteral("x (m)"), group), 1, 2);
    layout->addWidget(mission_x_spin_, 1, 3);
    layout->addWidget(new QLabel(QStringLiteral("y (m)"), group), 2, 0);
    layout->addWidget(mission_y_spin_, 2, 1);
    layout->addWidget(use_selected_button_, 2, 2);
    layout->addWidget(send_mission_button_, 2, 3);
    layout->addWidget(mission_status_label_, 3, 0, 1, 4);
    return group;
  }

  QWidget * build_arm_event_group(QWidget * parent)
  {
    auto * group = new QGroupBox(QStringLiteral("Arm Event Server"), parent);
    auto * layout = new QFormLayout(group);

    arm_event_success_check_ = new QCheckBox(QStringLiteral("success"), group);
    arm_event_success_check_->setChecked(true);
    arm_event_message_edit_ = new QLineEdit(QStringLiteral("ack"), group);
    last_arm_event_request_label_ = new QLabel(QStringLiteral("none"), group);

    layout->addRow(QStringLiteral("Response"), arm_event_success_check_);
    layout->addRow(QStringLiteral("Response Message"), arm_event_message_edit_);
    layout->addRow(QStringLiteral("Last Request"), last_arm_event_request_label_);
    return group;
  }

  QWidget * build_debug_group(QWidget * parent)
  {
    auto * group = new QGroupBox(QStringLiteral("Debug Mirrors"), parent);
    auto * layout = new QFormLayout(group);

    debug_mission_request_label_ = new QLabel(QStringLiteral("none"), group);
    debug_mission_response_label_ = new QLabel(QStringLiteral("none"), group);
    debug_arm_event_request_label_ = new QLabel(QStringLiteral("none"), group);
    debug_arm_event_response_label_ = new QLabel(QStringLiteral("none"), group);
    target_pose_label_ = new QLabel(QStringLiteral("none"), group);

    layout->addRow(QStringLiteral("Mission Req"), debug_mission_request_label_);
    layout->addRow(QStringLiteral("Mission Rsp"), debug_mission_response_label_);
    layout->addRow(QStringLiteral("Arm Event Req"), debug_arm_event_request_label_);
    layout->addRow(QStringLiteral("Arm Event Rsp"), debug_arm_event_response_label_);
    layout->addRow(QStringLiteral("Target Pose"), target_pose_label_);
    return group;
  }

  QWidget * build_task_points_group(QWidget * parent)
  {
    auto * group = new QGroupBox(QStringLiteral("Task Points"), parent);
    auto * layout = new QVBoxLayout(group);

    task_points_table_ = new QTableWidget(group);
    task_points_table_->setColumnCount(3);
    task_points_table_->setHorizontalHeaderLabels(
      {QStringLiteral("id"), QStringLiteral("x"), QStringLiteral("y")});
    task_points_table_->horizontalHeader()->setStretchLastSection(true);
    task_points_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    task_points_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    task_points_table_->setSelectionMode(QAbstractItemView::SingleSelection);

    auto * buttons = new QHBoxLayout();
    add_task_point_button_ = new QPushButton(QStringLiteral("Add"), group);
    remove_task_point_button_ = new QPushButton(QStringLiteral("Remove Selected"), group);
    load_defaults_button_ = new QPushButton(QStringLiteral("Load Defaults"), group);
    publish_task_points_button_ = new QPushButton(QStringLiteral("Publish Now"), group);

    buttons->addWidget(add_task_point_button_);
    buttons->addWidget(remove_task_point_button_);
    buttons->addWidget(load_defaults_button_);
    buttons->addWidget(publish_task_points_button_);

    layout->addLayout(buttons);
    layout->addWidget(task_points_table_, 1);
    return group;
  }

  QWidget * build_logs_group(QWidget * parent)
  {
    auto * group = new QGroupBox(QStringLiteral("Logs"), parent);
    auto * layout = new QVBoxLayout(group);
    logs_view_ = new QPlainTextEdit(group);
    logs_view_->setReadOnly(true);
    logs_view_->setMaximumBlockCount(kMaxLogLines);
    layout->addWidget(logs_view_, 1);
    return group;
  }

  static QDoubleSpinBox * make_double_spin(
    double min_value,
    double max_value,
    int decimals,
    double step,
    QWidget * parent)
  {
    auto * spin = new QDoubleSpinBox(parent);
    spin->setRange(min_value, max_value);
    spin->setDecimals(decimals);
    spin->setSingleStep(step);
    return spin;
  }

  void connect_signals()
  {
    connect(pose_x_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
      push_pose_to_node(false);
    });
    connect(pose_y_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
      push_pose_to_node(false);
    });
    connect(pose_yaw_deg_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
      push_pose_to_node(false);
    });
    connect(pose_publish_hz_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
      node_->set_pose_publish_hz(value);
    });
    connect(auto_publish_check_, &QCheckBox::toggled, this, [this](bool checked) {
      node_->set_pose_publish_enabled(checked);
    });
    connect(publish_pose_button_, &QPushButton::clicked, this, [this]() {
      push_pose_to_node(true);
    });

    connect(arm_event_success_check_, &QCheckBox::toggled, this, [this](bool) {
      push_arm_event_response_config();
    });
    connect(arm_event_message_edit_, &QLineEdit::textChanged, this, [this](const QString &) {
      push_arm_event_response_config();
    });

    connect(use_selected_button_, &QPushButton::clicked, this, [this]() {
      fill_mission_from_selected_point();
    });
    connect(send_mission_button_, &QPushButton::clicked, this, [this]() {
      node_->send_mission(
        action_combo_->currentText().toStdString(),
        static_cast<std::uint32_t>(task_index_spin_->value()),
        point_id_spin_->value(),
        mission_x_spin_->value(),
        mission_y_spin_->value());
    });

    connect(add_task_point_button_, &QPushButton::clicked, this, [this]() {
      adding_table_rows_ = true;
      const int row = task_points_table_->rowCount();
      task_points_table_->insertRow(row);
      task_points_table_->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));
      task_points_table_->setItem(row, 1, new QTableWidgetItem(QStringLiteral("0.000")));
      task_points_table_->setItem(row, 2, new QTableWidgetItem(QStringLiteral("0.000")));
      adding_table_rows_ = false;
      push_task_points_table_to_node();
      task_points_table_->selectRow(row);
    });
    connect(remove_task_point_button_, &QPushButton::clicked, this, [this]() {
      const auto ranges = task_points_table_->selectedRanges();
      if (ranges.isEmpty()) {
        return;
      }
      adding_table_rows_ = true;
      task_points_table_->removeRow(ranges.first().topRow());
      adding_table_rows_ = false;
      push_task_points_table_to_node();
    });
    connect(load_defaults_button_, &QPushButton::clicked, this, [this]() {
      load_task_points_into_table(node_->default_task_points());
      push_task_points_table_to_node();
    });
    connect(publish_task_points_button_, &QPushButton::clicked, this, [this]() {
      push_task_points_table_to_node();
      node_->publish_task_points();
    });
    connect(task_points_table_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem *) {
      if (adding_table_rows_) {
        return;
      }
      push_task_points_table_to_node();
    });
    connect(task_points_table_, &QTableWidget::itemSelectionChanged, this, [this]() {
      sync_selected_task_point_from_table();
    });

    map_view_->set_task_point_selected_callback([this](int point_id) {
      select_task_point_by_id(point_id);
    });
    map_view_->set_pose_set_callback([this](double x, double y) {
      {
        QSignalBlocker block_x(pose_x_spin_);
        QSignalBlocker block_y(pose_y_spin_);
        pose_x_spin_->setValue(x);
        pose_y_spin_->setValue(y);
      }
      push_pose_to_node(true);
    });
  }

  void load_task_points_into_table(const std::vector<TaskPointState> & points)
  {
    adding_table_rows_ = true;
    task_points_table_->setRowCount(0);
    for (const auto & point : points) {
      const int row = task_points_table_->rowCount();
      task_points_table_->insertRow(row);
      task_points_table_->setItem(row, 0, new QTableWidgetItem(QString::number(point.id)));
      task_points_table_->setItem(
        row, 1, new QTableWidgetItem(QString::number(point.x, 'f', 3)));
      task_points_table_->setItem(
        row, 2, new QTableWidgetItem(QString::number(point.y, 'f', 3)));
    }
    adding_table_rows_ = false;
  }

  void push_pose_to_node(bool publish_now)
  {
    node_->set_pose(
      pose_x_spin_->value(),
      pose_y_spin_->value(),
      pose_yaw_deg_spin_->value() * kDegToRad);
    if (publish_now) {
      node_->publish_pose_once();
    }
  }

  void push_arm_event_response_config()
  {
    node_->set_arm_event_response(
      arm_event_success_check_->isChecked(),
      arm_event_message_edit_->text().toStdString());
  }

  void push_task_points_table_to_node()
  {
    std::vector<TaskPointState> points;
    points.reserve(task_points_table_->rowCount());
    for (int row = 0; row < task_points_table_->rowCount(); ++row) {
      TaskPointState point;
      point.id = table_int(row, 0, row + 1);
      point.x = table_double(row, 1, 0.0);
      point.y = table_double(row, 2, 0.0);
      points.push_back(point);
    }
    node_->set_task_points(points);
  }

  int table_int(int row, int column, int fallback) const
  {
    const auto * item = task_points_table_->item(row, column);
    bool ok = false;
    const int value = item ? item->text().toInt(&ok) : fallback;
    return ok ? value : fallback;
  }

  double table_double(int row, int column, double fallback) const
  {
    const auto * item = task_points_table_->item(row, column);
    bool ok = false;
    const double value = item ? item->text().toDouble(&ok) : fallback;
    return ok ? value : fallback;
  }

  void sync_selected_task_point_from_table()
  {
    const auto ranges = task_points_table_->selectedRanges();
    if (ranges.isEmpty()) {
      selected_task_point_id_ = 0;
      has_selected_task_point_ = false;
      return;
    }
    const int row = ranges.first().topRow();
    selected_task_point_id_ = table_int(row, 0, 0);
    has_selected_task_point_ = true;
    fill_mission_from_selected_point();
  }

  void select_task_point_by_id(int point_id)
  {
    for (int row = 0; row < task_points_table_->rowCount(); ++row) {
      if (table_int(row, 0, 0) == point_id) {
        task_points_table_->selectRow(row);
        return;
      }
    }
  }

  void fill_mission_from_selected_point()
  {
    if (!has_selected_task_point_) {
      return;
    }

    for (int row = 0; row < task_points_table_->rowCount(); ++row) {
      if (table_int(row, 0, 0) != selected_task_point_id_) {
        continue;
      }
      task_index_spin_->setValue(selected_task_point_id_);
      point_id_spin_->setValue(selected_task_point_id_);
      mission_x_spin_->setValue(table_double(row, 1, 0.0));
      mission_y_spin_->setValue(table_double(row, 2, 0.0));
      return;
    }
  }

  void refresh_from_snapshot()
  {
    const auto snapshot = node_->snapshot();

    if (!ui_pose_synced_once_) {
      QSignalBlocker block_x(pose_x_spin_);
      QSignalBlocker block_y(pose_y_spin_);
      QSignalBlocker block_yaw(pose_yaw_deg_spin_);
      QSignalBlocker block_hz(pose_publish_hz_spin_);
      QSignalBlocker block_auto(auto_publish_check_);
      pose_x_spin_->setValue(snapshot.pose_x);
      pose_y_spin_->setValue(snapshot.pose_y);
      pose_yaw_deg_spin_->setValue(snapshot.pose_yaw_rad * kRadToDeg);
      pose_publish_hz_spin_->setValue(snapshot.pose_publish_hz);
      auto_publish_check_->setChecked(snapshot.pose_auto_publish);
      ui_pose_synced_once_ = true;
    }

    const QString status = QStringLiteral(
      "mission_service=%1 | pose=%2 | points=%3 | arm_event_response=%4")
      .arg(snapshot.mission_service_ready ? QStringLiteral("up") : QStringLiteral("down"))
      .arg(QString::fromStdString(format_pose(snapshot.pose_x, snapshot.pose_y, snapshot.pose_yaw_rad)))
      .arg(snapshot.task_points.size())
      .arg(snapshot.arm_event_response_success ? QStringLiteral("success") : QStringLiteral("fail"));
    status_label_->setText(status);

    mission_status_label_->setText(QString::fromStdString(snapshot.last_mission_call_status));

    last_arm_event_request_label_->setText(
      snapshot.has_last_arm_event_request ?
      QString::fromStdString(snapshot.last_arm_event_request) :
      QStringLiteral("none"));

    debug_mission_request_label_->setText(
      snapshot.has_debug_mission_request ?
      QString::fromStdString(snapshot.debug_mission_request) :
      QStringLiteral("none"));
    debug_mission_response_label_->setText(
      snapshot.has_debug_mission_response ?
      QString::fromStdString(snapshot.debug_mission_response) :
      QStringLiteral("none"));
    debug_arm_event_request_label_->setText(
      snapshot.has_debug_arm_event_request ?
      QString::fromStdString(snapshot.debug_arm_event_request) :
      QStringLiteral("none"));
    debug_arm_event_response_label_->setText(
      snapshot.has_debug_arm_event_response ?
      QString::fromStdString(snapshot.debug_arm_event_response) :
      QStringLiteral("none"));

    if (snapshot.has_target_pose) {
      target_pose_label_->setText(
        QString::fromStdString(
          format_pose(
            snapshot.target_pose.position.x,
            snapshot.target_pose.position.y,
            yaw_from_quaternion_wxyz(
              snapshot.target_pose.orientation.w,
              snapshot.target_pose.orientation.x,
              snapshot.target_pose.orientation.y,
              snapshot.target_pose.orientation.z))));
    } else {
      target_pose_label_->setText(QStringLiteral("none"));
    }

    NavMapView::DrawState draw_state;
    draw_state.task_points = snapshot.task_points;
    draw_state.pose_x = snapshot.pose_x;
    draw_state.pose_y = snapshot.pose_y;
    draw_state.pose_yaw_rad = snapshot.pose_yaw_rad;
    draw_state.has_target_pose = snapshot.has_target_pose;
    draw_state.target_pose = snapshot.target_pose;
    draw_state.has_selection = has_selected_task_point_;
    draw_state.selected_task_point_id = selected_task_point_id_;
    map_view_->set_draw_state(draw_state);

    if (snapshot.logs.size() != last_log_line_count_) {
      QStringList lines;
      for (const auto & line : snapshot.logs) {
        lines.push_back(QString::fromStdString(line));
      }
      logs_view_->setPlainText(lines.join('\n'));
      logs_view_->verticalScrollBar()->setValue(logs_view_->verticalScrollBar()->maximum());
      last_log_line_count_ = snapshot.logs.size();
    }
  }

  std::shared_ptr<NavMockRosNode> node_;
  QLabel * status_label_{nullptr};
  NavMapView * map_view_{nullptr};
  QDoubleSpinBox * pose_x_spin_{nullptr};
  QDoubleSpinBox * pose_y_spin_{nullptr};
  QDoubleSpinBox * pose_yaw_deg_spin_{nullptr};
  QDoubleSpinBox * pose_publish_hz_spin_{nullptr};
  QCheckBox * auto_publish_check_{nullptr};
  QPushButton * publish_pose_button_{nullptr};
  QLabel * pose_note_label_{nullptr};
  QComboBox * action_combo_{nullptr};
  QSpinBox * task_index_spin_{nullptr};
  QSpinBox * point_id_spin_{nullptr};
  QDoubleSpinBox * mission_x_spin_{nullptr};
  QDoubleSpinBox * mission_y_spin_{nullptr};
  QPushButton * use_selected_button_{nullptr};
  QPushButton * send_mission_button_{nullptr};
  QLabel * mission_status_label_{nullptr};
  QCheckBox * arm_event_success_check_{nullptr};
  QLineEdit * arm_event_message_edit_{nullptr};
  QLabel * last_arm_event_request_label_{nullptr};
  QLabel * debug_mission_request_label_{nullptr};
  QLabel * debug_mission_response_label_{nullptr};
  QLabel * debug_arm_event_request_label_{nullptr};
  QLabel * debug_arm_event_response_label_{nullptr};
  QLabel * target_pose_label_{nullptr};
  QTableWidget * task_points_table_{nullptr};
  QPushButton * add_task_point_button_{nullptr};
  QPushButton * remove_task_point_button_{nullptr};
  QPushButton * load_defaults_button_{nullptr};
  QPushButton * publish_task_points_button_{nullptr};
  QPlainTextEdit * logs_view_{nullptr};
  QTimer * refresh_timer_{nullptr};
  bool ui_pose_synced_once_{false};
  bool adding_table_rows_{false};
  bool has_selected_task_point_{false};
  int selected_task_point_id_{0};
  std::size_t last_log_line_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<NavMockRosNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  NavMockWindow window(node);
  window.show();

  const int rc = app.exec();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return rc;
}
