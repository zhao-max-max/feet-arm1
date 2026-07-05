#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <QApplication>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSpinBox>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "navigation/srv/mission_command.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr int kMaxLogLines = 200;

std::string format_mission_request(const navigation::srv::MissionCommand::Request & request)
{
  std::ostringstream out;
  out << request.action
      << " idx=" << request.task_index
      << " point=" << request.point_id
      << " map=(" << std::fixed << std::setprecision(3) << request.x << ", " << request.y << ")";
  return out.str();
}

std::string format_mission_response(const navigation::srv::MissionCommand::Response & response)
{
  std::ostringstream out;
  out << (response.success ? "ok" : "fail") << ": " << response.message;
  return out.str();
}
}  // namespace

class NavArmStateRosNode : public rclcpp::Node
{
public:
  struct Snapshot
  {
    bool debug_state_service_ready{false};
    std::uint32_t initial_task_index{1};
    std::string arm_debug_state_service;
    std::string last_debug_state_call_status{"idle"};
    std::string last_request_summary{"none"};
    std::deque<std::string> logs;
  };

  NavArmStateRosNode()
  : Node("nav_arm_state_ui_node")
  {
    arm_debug_state_service_ =
      declare_parameter<std::string>("arm_debug_state_service", "/arm/debug_state_command");
    initial_task_index_ = static_cast<std::uint32_t>(
      std::max<int64_t>(0, declare_parameter<int64_t>("initial_task_index", 1)));

    arm_debug_state_client_ =
      create_client<navigation::srv::MissionCommand>(arm_debug_state_service_);

    append_log("nav arm state UI ready: arm_debug_state_service=" + arm_debug_state_service_);
  }

  Snapshot snapshot() const
  {
    Snapshot out;
    std::lock_guard<std::mutex> lock(mutex_);
    out.debug_state_service_ready = arm_debug_state_client_->service_is_ready();
    out.initial_task_index = initial_task_index_;
    out.arm_debug_state_service = arm_debug_state_service_;
    out.last_debug_state_call_status = last_debug_state_call_status_;
    out.last_request_summary = last_request_summary_;
    out.logs = logs_;
    return out;
  }

  std::uint32_t initial_task_index() const
  {
    return initial_task_index_;
  }

  void send_debug_state(const std::string & action, std::uint32_t task_index)
  {
    if (!arm_debug_state_client_->service_is_ready()) {
      append_log("debug state service not ready: " + arm_debug_state_service_);
      std::lock_guard<std::mutex> lock(mutex_);
      last_debug_state_call_status_ = "service not ready";
      return;
    }

    auto request = std::make_shared<navigation::srv::MissionCommand::Request>();
    request->action = action;
    request->task_index = task_index;
    request->point_id = static_cast<int>(task_index);
    request->x = 0.0;
    request->y = 0.0;

    const std::string request_text = format_mission_request(*request);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_request_summary_ = request_text;
      last_debug_state_call_status_ = "requesting " + request_text;
    }
    append_log("debug state request -> " + request_text);

    arm_debug_state_client_->async_send_request(
      request,
      [this, action](rclcpp::Client<navigation::srv::MissionCommand>::SharedFuture future) {
        const auto response = future.get();
        const std::string text = response ?
          format_mission_response(*response) : std::string("null response");
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_debug_state_call_status_ = action + " => " + text;
        }
        append_log("debug state response <- " + text);
      });
  }

private:
  void append_log(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    logs_.push_back(message);
    while (static_cast<int>(logs_.size()) > kMaxLogLines) {
      logs_.pop_front();
    }
  }

  std::string arm_debug_state_service_;
  std::uint32_t initial_task_index_{1};

  mutable std::mutex mutex_;
  std::string last_debug_state_call_status_{"idle"};
  std::string last_request_summary_{"none"};
  std::deque<std::string> logs_;

  rclcpp::Client<navigation::srv::MissionCommand>::SharedPtr arm_debug_state_client_;
};

class NavArmStateWindow : public QMainWindow
{
public:
  explicit NavArmStateWindow(std::shared_ptr<NavArmStateRosNode> node)
  : node_(std::move(node))
  {
    setWindowTitle(QStringLiteral("Nav Arm State Debug Tool"));
    resize(760, 420);

    auto * central = new QWidget(this);
    auto * root_layout = new QVBoxLayout(central);

    auto * info_group = new QGroupBox(QStringLiteral("Service Status"), central);
    auto * info_layout = new QVBoxLayout(info_group);
    service_label_ = new QLabel(QStringLiteral("service: --"), info_group);
    status_label_ = new QLabel(QStringLiteral("idle"), info_group);
    info_layout->addWidget(service_label_);
    info_layout->addWidget(status_label_);
    root_layout->addWidget(info_group);

    auto * control_group = new QGroupBox(QStringLiteral("Arm State Debug"), central);
    auto * control_layout = new QGridLayout(control_group);
    task_index_spin_ = new QSpinBox(control_group);
    task_index_spin_->setRange(0, 1000000);
    task_index_spin_->setValue(static_cast<int>(node_->initial_task_index()));
    task_hint_label_ = new QLabel(
      QStringLiteral("LOOKOUT uses task_index to compute the target. Set 0 to reuse cached ready target."),
      control_group);
    task_hint_label_->setWordWrap(true);

    idle_button_ = new QPushButton(QStringLiteral("Set IDLE"), control_group);
    lookout_button_ = new QPushButton(QStringLiteral("Move LOOKOUT"), control_group);
    holding_button_ = new QPushButton(QStringLiteral("Set HOLDING"), control_group);
    carry_holding_button_ = new QPushButton(QStringLiteral("Carry + HOLDING"), control_group);
    last_request_label_ = new QLabel(QStringLiteral("last request: none"), control_group);
    last_request_label_->setWordWrap(true);

    control_layout->addWidget(new QLabel(QStringLiteral("task_index"), control_group), 0, 0);
    control_layout->addWidget(task_index_spin_, 0, 1);
    control_layout->addWidget(idle_button_, 1, 0);
    control_layout->addWidget(lookout_button_, 1, 1);
    control_layout->addWidget(holding_button_, 1, 2);
    control_layout->addWidget(carry_holding_button_, 1, 3);
    control_layout->addWidget(task_hint_label_, 2, 0, 1, 4);
    control_layout->addWidget(last_request_label_, 3, 0, 1, 4);
    root_layout->addWidget(control_group);

    auto * logs_group = new QGroupBox(QStringLiteral("Logs"), central);
    auto * logs_layout = new QVBoxLayout(logs_group);
    logs_view_ = new QPlainTextEdit(logs_group);
    logs_view_->setReadOnly(true);
    logs_view_->setMaximumBlockCount(kMaxLogLines);
    logs_layout->addWidget(logs_view_);
    root_layout->addWidget(logs_group, 1);

    setCentralWidget(central);

    connect(idle_button_, &QPushButton::clicked, this, [this]() {
      node_->send_debug_state("idle", selected_task_index());
    });
    connect(lookout_button_, &QPushButton::clicked, this, [this]() {
      node_->send_debug_state("lookout", selected_task_index());
    });
    connect(holding_button_, &QPushButton::clicked, this, [this]() {
      node_->send_debug_state("holding", selected_task_index());
    });
    connect(carry_holding_button_, &QPushButton::clicked, this, [this]() {
      node_->send_debug_state("carry_holding", selected_task_index());
    });

    poll_timer_ = new QTimer(this);
    poll_timer_->setInterval(100);
    connect(poll_timer_, &QTimer::timeout, this, [this]() {
      refresh_snapshot();
    });
    poll_timer_->start();
    refresh_snapshot();
  }

private:
  std::uint32_t selected_task_index() const
  {
    return static_cast<std::uint32_t>(task_index_spin_->value());
  }

  void refresh_snapshot()
  {
    const auto snapshot = node_->snapshot();

    service_label_->setText(
      QStringLiteral("service=%1 (%2)")
      .arg(QString::fromStdString(snapshot.arm_debug_state_service))
      .arg(snapshot.debug_state_service_ready ? QStringLiteral("up") : QStringLiteral("down")));
    status_label_->setText(QString::fromStdString(snapshot.last_debug_state_call_status));
    last_request_label_->setText(
      QStringLiteral("last request: %1").arg(QString::fromStdString(snapshot.last_request_summary)));

    QStringList lines;
    lines.reserve(static_cast<int>(snapshot.logs.size()));
    for (const auto & line : snapshot.logs) {
      lines.push_back(QString::fromStdString(line));
    }
    const QString log_text = lines.join(QLatin1Char('\n'));
    if (logs_view_->toPlainText() != log_text) {
      logs_view_->setPlainText(log_text);
      auto * bar = logs_view_->verticalScrollBar();
      if (bar != nullptr) {
        bar->setValue(bar->maximum());
      }
    }
  }

  std::shared_ptr<NavArmStateRosNode> node_;
  QLabel * service_label_{nullptr};
  QLabel * status_label_{nullptr};
  QSpinBox * task_index_spin_{nullptr};
  QLabel * task_hint_label_{nullptr};
  QPushButton * idle_button_{nullptr};
  QPushButton * lookout_button_{nullptr};
  QPushButton * holding_button_{nullptr};
  QPushButton * carry_holding_button_{nullptr};
  QLabel * last_request_label_{nullptr};
  QPlainTextEdit * logs_view_{nullptr};
  QTimer * poll_timer_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<NavArmStateRosNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  NavArmStateWindow window(node);
  window.show();

  const int rc = app.exec();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return rc;
}
