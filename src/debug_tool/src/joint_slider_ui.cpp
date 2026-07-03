#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_joint.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/srv/set_controller_mode.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr int kJointCount = 5;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr int kSliderScale = 100;

using JointArray = std::array<double, kJointCount>;

JointArray to_joint_array(const std::vector<double> & values, const JointArray & fallback)
{
  if (values.size() != kJointCount) {
    return fallback;
  }

  JointArray out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

std::string format_deg(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(1) << value;
  return out.str();
}
}  // namespace

class JointSliderRosNode : public rclcpp::Node
{
public:
  using MoveJoint = robot_msgs::action::MoveJoint;
  using GoalHandleMoveJoint = rclcpp_action::ClientGoalHandle<MoveJoint>;

  struct Snapshot
  {
    bool has_state{false};
    bool has_ready{false};
    bool driver_ready{false};
    bool action_ready{false};
    bool mode_service_ready{false};
    double state_age_sec{-1.0};
    double ready_age_sec{-1.0};
    JointArray q_deg{};
    JointArray dq{};
    std::array<bool, kJointCount> valid{};
    std::string last_action_status;
  };

  JointSliderRosNode()
  : Node("joint_slider_ui_node")
  {
    state_topic_ = declare_parameter<std::string>("state_topic", "/arm2/_lowState/joint");
    ready_topic_ = declare_parameter<std::string>("ready_topic", "/robot_driver/ready");
    action_name_ = declare_parameter<std::string>("action_name", "move_joint");
    mode_service_name_ =
      declare_parameter<std::string>("mode_service", "set_controller_mode");
    max_velocity_ = declare_parameter<double>("max_velocity", 0.6);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 1.0);
    blend_radius_ = declare_parameter<double>("blend_radius", 0.02);
    send_debounce_ms_ = declare_parameter<int>("send_debounce_ms", 180);
    controller_modes_ = declare_parameter<std::vector<std::string>>(
      "controller_modes", {"idle", "gravity_comp", "moving", "loaded"});
    default_mode_ = declare_parameter<std::string>("default_mode", "moving");

    const JointArray default_lower{-240.0, 0.0, -172.0, -115.0, -180.0};
    const JointArray default_upper{240.0, 200.0, 10.0, 90.0, 180.0};
    lower_deg_ =
      to_joint_array(declare_parameter<std::vector<double>>(
        "joint_lower_deg",
        std::vector<double>(default_lower.begin(), default_lower.end())),
      default_lower);
    upper_deg_ =
      to_joint_array(declare_parameter<std::vector<double>>(
        "joint_upper_deg",
        std::vector<double>(default_upper.begin(), default_upper.end())),
      default_upper);

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = create_subscription<robot_msgs::msg::RobotState>(
      state_topic_, state_qos,
      [this](const robot_msgs::msg::RobotState::SharedPtr msg) {
        if (!msg || msg->motor_state.size() < kJointCount) {
          return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < kJointCount; ++i) {
          q_deg_[i] = msg->motor_state[i].q * kRadToDeg;
          dq_[i] = msg->motor_state[i].dq;
          valid_[i] = msg->motor_state[i].valid;
        }
        has_state_ = true;
        last_state_time_ = now();
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
        has_ready_ = true;
        last_ready_time_ = now();
      });

    mode_client_ = create_client<robot_msgs::srv::SetControllerMode>(mode_service_name_);
    action_client_ = rclcpp_action::create_client<MoveJoint>(this, action_name_);

    RCLCPP_INFO(
      get_logger(),
      "joint_slider_ui_node started. state=%s ready=%s action=%s mode_service=%s",
      state_topic_.c_str(), ready_topic_.c_str(), action_name_.c_str(),
      mode_service_name_.c_str());
  }

  Snapshot snapshot()
  {
    Snapshot out;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      out.has_state = has_state_;
      out.has_ready = has_ready_;
      out.driver_ready = driver_ready_;
      out.q_deg = q_deg_;
      out.dq = dq_;
      out.valid = valid_;
      out.last_action_status = last_action_status_;
      const auto t_now = now();
      out.state_age_sec = has_state_ ? (t_now - last_state_time_).seconds() : -1.0;
      out.ready_age_sec = has_ready_ ? (t_now - last_ready_time_).seconds() : -1.0;
    }
    out.action_ready = action_client_->action_server_is_ready();
    out.mode_service_ready = mode_client_->service_is_ready();
    return out;
  }

  const JointArray & lower_deg() const
  {
    return lower_deg_;
  }

  const JointArray & upper_deg() const
  {
    return upper_deg_;
  }

  int send_debounce_ms() const
  {
    return std::max(20, send_debounce_ms_);
  }

  const std::vector<std::string> & controller_modes() const
  {
    return controller_modes_;
  }

  const std::string & default_mode() const
  {
    return default_mode_;
  }

  void request_controller_mode(const std::string & mode)
  {
    if (!mode_client_->service_is_ready()) {
      set_status("set_controller_mode service is not ready");
      return;
    }

    auto request = std::make_shared<robot_msgs::srv::SetControllerMode::Request>();
    request->mode = mode;
    mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedFuture future) {
        const auto response = future.get();
        if (response && response->success) {
          set_status("controller mode switched to " + mode);
        } else {
          set_status(response ? response->message : "mode switch failed");
        }
      });
  }

  bool send_target_deg(const JointArray & target_deg)
  {
    if (!action_client_->action_server_is_ready()) {
      set_status("move_joint action server is not ready");
      return false;
    }

    MoveJoint::Goal goal;
    goal.num_points = 1;
    goal.max_velocity = max_velocity_;
    goal.max_acceleration = max_acceleration_;
    goal.blend_radius = blend_radius_;
    goal.joint_targets.reserve(kJointCount);
    for (int i = 0; i < kJointCount; ++i) {
      const double target = clamp(target_deg[i], lower_deg_[i], upper_deg_[i]);
      goal.joint_targets.push_back(target * kDegToRad);
    }

    auto options = rclcpp_action::Client<MoveJoint>::SendGoalOptions();
    options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_goal_handle_ = goal_handle;
        last_action_status_ = goal_handle ? "goal accepted" : "goal rejected";
      };
    options.result_callback =
      [this](const GoalHandleMoveJoint::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success)
        {
          last_action_status_ = "goal succeeded";
        } else if (result.result) {
          last_action_status_ = "goal failed: " + result.result->message;
        } else {
          last_action_status_ = "goal failed";
        }
      };

    action_client_->async_send_goal(goal, options);
    set_status("goal sent");
    return true;
  }

  void cancel_active_goal()
  {
    std::shared_ptr<GoalHandleMoveJoint> goal_handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle = active_goal_handle_;
    }
    if (goal_handle) {
      action_client_->async_cancel_goal(goal_handle);
      set_status("active goal cancel requested");
    }
  }

private:
  void set_status(const std::string & text)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_action_status_ = text;
  }

  std::string state_topic_;
  std::string ready_topic_;
  std::string action_name_;
  std::string mode_service_name_;
  double max_velocity_{0.6};
  double max_acceleration_{1.0};
  double blend_radius_{0.02};
  int send_debounce_ms_{180};
  std::vector<std::string> controller_modes_;
  std::string default_mode_{"moving"};
  JointArray lower_deg_{};
  JointArray upper_deg_{};

  std::mutex mutex_;
  bool has_state_{false};
  bool has_ready_{false};
  bool driver_ready_{false};
  JointArray q_deg_{};
  JointArray dq_{};
  std::array<bool, kJointCount> valid_{};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ready_time_{0, 0, RCL_ROS_TIME};
  std::string last_action_status_{"idle"};
  std::shared_ptr<GoalHandleMoveJoint> active_goal_handle_;

  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Client<robot_msgs::srv::SetControllerMode>::SharedPtr mode_client_;
  rclcpp_action::Client<MoveJoint>::SharedPtr action_client_;
};

class JointSliderWindow : public QMainWindow
{
public:
  explicit JointSliderWindow(std::shared_ptr<JointSliderRosNode> node)
  : node_(std::move(node))
  {
    setWindowTitle(QStringLiteral("Arm Joint Slider Debug Tool"));
    resize(980, 440);

    const auto & lower = node_->lower_deg();
    const auto & upper = node_->upper_deg();
    target_deg_ = lower;
    for (int i = 0; i < kJointCount; ++i) {
      target_deg_[i] = 0.0;
    }

    auto * central = new QWidget(this);
    auto * root_layout = new QVBoxLayout(central);

    auto * top_layout = new QHBoxLayout();
    mode_combo_ = new QComboBox(central);
    for (const auto & mode : node_->controller_modes()) {
      mode_combo_->addItem(QString::fromStdString(mode));
    }
    const int default_mode_index =
      mode_combo_->findText(QString::fromStdString(node_->default_mode()));
    if (default_mode_index >= 0) {
      mode_combo_->setCurrentIndex(default_mode_index);
    }
    mode_combo_->setMinimumHeight(36);
    mode_button_ = new QPushButton(QStringLiteral("切换参数模式"), central);
    mode_button_->setMinimumHeight(36);
    motion_switch_ = new QPushButton(QStringLiteral("运动关闭"), central);
    motion_switch_->setCheckable(true);
    motion_switch_->setMinimumHeight(36);
    sync_button_ = new QPushButton(QStringLiteral("同步当前关节角"), central);
    sync_button_->setMinimumHeight(36);
    status_label_ = new QLabel(QStringLiteral("waiting for ROS state"), central);
    status_label_->setMinimumHeight(36);

    top_layout->addWidget(new QLabel(QStringLiteral("参数模式"), central));
    top_layout->addWidget(mode_combo_);
    top_layout->addWidget(mode_button_);
    top_layout->addWidget(motion_switch_);
    top_layout->addWidget(sync_button_);
    top_layout->addWidget(status_label_, 1);
    root_layout->addLayout(top_layout);

    auto * group = new QGroupBox(QStringLiteral("Joint Targets (deg)"), central);
    auto * grid = new QGridLayout(group);
    grid->addWidget(new QLabel(QStringLiteral("Axis"), group), 0, 0);
    grid->addWidget(new QLabel(QStringLiteral("Target"), group), 0, 1);
    grid->addWidget(new QLabel(QStringLiteral("Value"), group), 0, 2);
    grid->addWidget(new QLabel(QStringLiteral("Actual"), group), 0, 3);
    grid->addWidget(new QLabel(QStringLiteral("Valid"), group), 0, 4);

    const std::array<QString, kJointCount> names{
      QStringLiteral("J0 yaw"),
      QStringLiteral("J1 pitch_1"),
      QStringLiteral("J2 pitch_2"),
      QStringLiteral("J3 pitch_3"),
      QStringLiteral("J4 roll")};

    for (int i = 0; i < kJointCount; ++i) {
      auto & row = rows_[i];
      row.name = new QLabel(names[i], group);
      row.slider = new QSlider(Qt::Horizontal, group);
      row.slider->setRange(
        static_cast<int>(std::round(lower[i] * kSliderScale)),
        static_cast<int>(std::round(upper[i] * kSliderScale)));
      row.slider->setSingleStep(10);
      row.slider->setPageStep(100);
      row.spin = new QDoubleSpinBox(group);
      row.spin->setRange(lower[i], upper[i]);
      row.spin->setDecimals(2);
      row.spin->setSingleStep(0.5);
      row.spin->setSuffix(QStringLiteral(" deg"));
      row.actual = new QLabel(QStringLiteral("--"), group);
      row.valid = new QLabel(QStringLiteral("--"), group);

      grid->addWidget(row.name, i + 1, 0);
      grid->addWidget(row.slider, i + 1, 1);
      grid->addWidget(row.spin, i + 1, 2);
      grid->addWidget(row.actual, i + 1, 3);
      grid->addWidget(row.valid, i + 1, 4);

      connect(row.slider, &QSlider::valueChanged, this, [this, i](int value) {
        if (updating_ui_) {
          return;
        }
        set_joint_target(i, static_cast<double>(value) / kSliderScale);
        schedule_send();
      });
      connect(row.slider, &QSlider::sliderReleased, this, [this]() {
        send_now();
      });
      connect(
        row.spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
        this, [this, i](double value) {
          if (updating_ui_) {
            return;
          }
          set_joint_target(i, value);
          schedule_send();
        });
    }

    root_layout->addWidget(group, 1);
    setCentralWidget(central);

    send_timer_ = new QTimer(this);
    send_timer_->setSingleShot(true);
    send_timer_->setInterval(node_->send_debounce_ms());
    connect(send_timer_, &QTimer::timeout, this, [this]() {
      send_now();
    });

    poll_timer_ = new QTimer(this);
    poll_timer_->setInterval(100);
    connect(poll_timer_, &QTimer::timeout, this, [this]() {
      refresh_snapshot();
    });
    poll_timer_->start();

    connect(motion_switch_, &QPushButton::toggled, this, [this](bool checked) {
      move_enabled_ = checked;
      motion_switch_->setText(checked ? QStringLiteral("运动开启") : QStringLiteral("运动关闭"));
      if (checked) {
        request_selected_mode();
        status_label_->setText(
          QStringLiteral("motion enabled; slider changes send goals"));
      } else {
        send_timer_->stop();
        node_->cancel_active_goal();
        status_label_->setText(QStringLiteral("motion disabled"));
      }
    });
    connect(mode_button_, &QPushButton::clicked, this, [this]() {
      request_selected_mode();
    });
    connect(sync_button_, &QPushButton::clicked, this, [this]() {
      sync_from_robot_state();
    });
  }

private:
  struct JointRow
  {
    QLabel * name{nullptr};
    QSlider * slider{nullptr};
    QDoubleSpinBox * spin{nullptr};
    QLabel * actual{nullptr};
    QLabel * valid{nullptr};
  };

  void set_joint_target(int index, double value_deg)
  {
    const auto & lower = node_->lower_deg();
    const auto & upper = node_->upper_deg();
    target_deg_[index] = clamp(value_deg, lower[index], upper[index]);

    QSignalBlocker block_slider(rows_[index].slider);
    QSignalBlocker block_spin(rows_[index].spin);
    rows_[index].slider->setValue(static_cast<int>(std::round(target_deg_[index] * kSliderScale)));
    rows_[index].spin->setValue(target_deg_[index]);
  }

  QString selected_mode() const
  {
    return mode_combo_ ? mode_combo_->currentText() : QStringLiteral("moving");
  }

  void request_selected_mode()
  {
    const auto mode = selected_mode();
    if (mode.isEmpty()) {
      status_label_->setText(QStringLiteral("no controller mode selected"));
      return;
    }
    node_->request_controller_mode(mode.toStdString());
    status_label_->setText(QStringLiteral("requesting mode: ") + mode);
  }

  void schedule_send()
  {
    if (!move_enabled_ || updating_ui_) {
      return;
    }
    send_timer_->start();
  }

  void send_now()
  {
    if (!move_enabled_ || updating_ui_) {
      return;
    }
    send_timer_->stop();
    const bool sent = node_->send_target_deg(target_deg_);
    if (sent) {
      status_label_->setText(QStringLiteral("goal sent"));
    } else {
      status_label_->setText(QStringLiteral("move_joint is not ready"));
    }
  }

  void sync_from_robot_state()
  {
    const auto snapshot = node_->snapshot();
    if (!snapshot.has_state) {
      status_label_->setText(QStringLiteral("no joint state to sync"));
      return;
    }

    updating_ui_ = true;
    for (int i = 0; i < kJointCount; ++i) {
      set_joint_target(i, snapshot.q_deg[i]);
    }
    updating_ui_ = false;
    status_label_->setText(QStringLiteral("synced sliders from robot state"));
  }

  void refresh_snapshot()
  {
    const auto snapshot = node_->snapshot();
    if (snapshot.has_state && !synced_once_) {
      sync_from_robot_state();
      synced_once_ = true;
    }

    for (int i = 0; i < kJointCount; ++i) {
      rows_[i].actual->setText(
        QString::fromStdString(format_deg(snapshot.q_deg[i])) + QStringLiteral(" deg"));
      rows_[i].valid->setText(snapshot.valid[i] ? QStringLiteral("1") : QStringLiteral("0"));
    }

    QString status = QStringLiteral("ready=%1 state=%2 action=%3 mode_srv=%4 age=%5s | %6")
      .arg(snapshot.driver_ready ? QStringLiteral("true") : QStringLiteral("false"))
      .arg(snapshot.has_state ? QStringLiteral("ok") : QStringLiteral("none"))
      .arg(snapshot.action_ready ? QStringLiteral("up") : QStringLiteral("down"))
      .arg(snapshot.mode_service_ready ? QStringLiteral("up") : QStringLiteral("down"))
      .arg(snapshot.state_age_sec, 0, 'f', 2)
      .arg(QString::fromStdString(snapshot.last_action_status));
    status_label_->setText(status);
  }

  std::shared_ptr<JointSliderRosNode> node_;
  std::array<JointRow, kJointCount> rows_;
  JointArray target_deg_{};
  QComboBox * mode_combo_{nullptr};
  QPushButton * mode_button_{nullptr};
  QPushButton * motion_switch_{nullptr};
  QPushButton * sync_button_{nullptr};
  QLabel * status_label_{nullptr};
  QTimer * send_timer_{nullptr};
  QTimer * poll_timer_{nullptr};
  bool move_enabled_{false};
  bool updating_ui_{false};
  bool synced_once_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<JointSliderRosNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  JointSliderWindow window(node);
  window.show();

  const int rc = app.exec();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return rc;
}
