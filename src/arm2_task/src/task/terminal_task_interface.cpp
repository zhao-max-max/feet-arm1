#include "arm2_task/task/terminal_task_interface.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <utility>
#include <unistd.h>

#include "geometry_msgs/msg/pose.hpp"

namespace arm2_task::task
{

TerminalTaskInterface::TerminalTaskInterface(
  rclcpp::Node * node,
  MotionClient * motion_client,
  EndEffectorClient * end_effector_client,
  TaskPrimitives * primitives,
  TaskSequences * sequences,
  ThreePhasePipeline * three_phase_pipeline,
  const std::map<std::string, Eigen::VectorXd> * presets,
  std::atomic<bool> * is_running,
  Config config)
: node_(node),
  motion_client_(motion_client),
  end_effector_client_(end_effector_client),
  primitives_(primitives),
  sequences_(sequences),
  three_phase_pipeline_(three_phase_pipeline),
  presets_(presets),
  is_running_(is_running),
  config_(std::move(config))
{
}

bool TerminalTaskInterface::wait_for_user_command(int * input_cmd)
{
  if (input_cmd == nullptr) {
    return false;
  }

  while (rclcpp::ok() && is_running_->load()) {
    pollfd stdin_poll{};
    stdin_poll.fd = STDIN_FILENO;
    stdin_poll.events = POLLIN;

    const int poll_rc = ::poll(&stdin_poll, 1, 200);
    if (poll_rc == 0) {
      continue;
    }
    if (poll_rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(node_->get_logger(), "poll(stdin) failed.");
      return false;
    }
    if ((stdin_poll.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      RCLCPP_INFO(node_->get_logger(), "stdin became unavailable; stopping task loop.");
      return false;
    }

    if (!(std::cin >> *input_cmd)) {
      if (std::cin.eof()) {
        RCLCPP_INFO(node_->get_logger(), "stdin closed; stopping task loop.");
        return false;
      }
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Please enter a valid number: " << std::flush;
      continue;
    }
    return true;
  }
  return false;
}

int TerminalTaskInterface::request_payload_estimate()
{
  double estimated_mass = 0.0;
  if (!end_effector_client_->request_payload_estimate(&estimated_mass)) {
    return 0;
  }
  last_estimated_mass_ = estimated_mass;
  return 1;
}

int TerminalTaskInterface::request_payload_state(bool has_load)
{
  return end_effector_client_->request_payload_state(
    has_load,
    config_.require_payload_service,
    config_.payload_default_has_load,
    config_.payload_default_mass,
    config_.payload_default_com);
}

void TerminalTaskInterface::run()
{
  RCLCPP_INFO(node_->get_logger(), "Task Node Ready.");

  while (rclcpp::ok() && is_running_->load()) {
    std::cout
      << "\n====== Task Control Panel ======\n"
      << "1:  Reset (suction OFF -> moving -> reset -> idle)\n"
      << "2:  Joint preset A (debug)\n"
      << "3:  Joint preset B (debug)\n"
      << "4:  Auto place   (dog camera perception -> pre-place -> place -> suction OFF -> retreat)\n"
      << "5:  Manual place (input frame x y z yaw -> pre-place -> place -> suction OFF -> retreat)\n"
      << "6:  Auto grasp   (perception/mock -> look_out -> grasp -> suction)\n"
      << "7:  Manual grasp (input world x y z -> look_out -> grasp -> suction)\n"
      << "8:  Release (suction OFF -> moving)\n"
      << "9:  Carry reset (suction ON, moving -> carry preset -> loaded)\n"
      << "10: Move to load preset\n"
      << "11: Estimate payload\n"
      << "12: 3-Phase Grasp (scan -> overhead align -> joint4 -90 -> grasp)\n"
      << "13: 3-Phase Place (scan -> overhead align -> joint4 -90 -> place)\n"
      << "14: Auto Stack   (dog camera -> pre-stack -> stack -> suction OFF -> retreat)\n"
      << "15: Manual Stack (input box top x y z yaw -> stack -> suction OFF -> retreat)\n"
      << "16: Enable payload model\n"
      << "17: Clear payload model\n"
      << "0:  Exit\n"
      << "cmd: " << std::flush;

    int input_cmd = 0;
    if (!wait_for_user_command(&input_cmd)) {
      return;
    }

    const auto cmd_start_time = std::chrono::steady_clock::now();
    Eigen::VectorXd q(5);

    switch (input_cmd) {
      case 1:
        RCLCPP_INFO(node_->get_logger(), ">>> Reset");
        primitives_->do_reset();
        break;

      case 2:
        RCLCPP_INFO(node_->get_logger(), ">>> Preset A");
        motion_client_->request_mode_switch("moving");
        q << 0, 160, -130, 40, 0;
        q *= M_PI / 180.0;
        if (motion_client_->send_move_goal(q)) {
          motion_client_->wait_for_action_completion();
        }
        break;

      case 3:
        RCLCPP_INFO(node_->get_logger(), ">>> Preset B");
        motion_client_->request_mode_switch("moving");
        q << 180, 90, -90, -90, 0;
        q *= M_PI / 180.0;
        if (motion_client_->send_move_goal(q)) {
          motion_client_->wait_for_action_completion();
        }
        break;

      case 4:
        RCLCPP_INFO(node_->get_logger(), ">>> Auto Place (dog camera perception)");
        if (!sequences_->place_mock_or_perception()) {
          RCLCPP_ERROR(node_->get_logger(), "[case4] Place move failed.");
          break;
        }
        RCLCPP_INFO(node_->get_logger(), "[case4] Auto Place done.");
        break;

      case 5:
      {
        RCLCPP_INFO(node_->get_logger(), ">>> Manual Place (input frame pose)");
        double tx = 0.0;
        double ty = 0.0;
        double tz = 0.0;
        double tyaw = 0.0;
        std::cout << "Enter frame center x y z (world, m) and yaw (rad): ";
        if (!(std::cin >> tx >> ty >> tz >> tyaw)) {
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          RCLCPP_WARN(node_->get_logger(), "Invalid input.");
          break;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        geometry_msgs::msg::Pose frame_pose;
        frame_pose.position.x = tx;
        frame_pose.position.y = ty;
        frame_pose.position.z = tz;
        frame_pose.orientation.z = std::sin(tyaw / 2.0);
        frame_pose.orientation.w = std::cos(tyaw / 2.0);

        RCLCPP_INFO(
          node_->get_logger(), "[case5] frame pos=(%.3f,%.3f,%.3f) yaw=%.3f",
          tx, ty, tz, tyaw);
        if (!sequences_->place_pose(frame_pose)) {
          RCLCPP_ERROR(node_->get_logger(), "[case5] Place move failed.");
          break;
        }
        RCLCPP_INFO(node_->get_logger(), "[case5] Manual Place done.");
        break;
      }

      case 6:
        RCLCPP_INFO(node_->get_logger(), ">>> Auto Grasp");
        sequences_->grasp_mock_or_perception();
        break;

      case 7:
      {
        double tx = 0.0;
        double ty = 0.0;
        double tz = 0.0;
        std::cout << "Enter target x y z (world frame, meters): ";
        if (!(std::cin >> tx >> ty >> tz)) {
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          RCLCPP_WARN(node_->get_logger(), "Invalid input.");
          break;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        geometry_msgs::msg::Pose target;
        target.position.x = tx;
        target.position.y = ty;
        target.position.z = tz;
        target.orientation.w = 1.0;
        RCLCPP_INFO(
          node_->get_logger(), ">>> Manual Grasp target=(%.3f,%.3f,%.3f)", tx, ty, tz);
        sequences_->grasp_pose(target, false);
        break;
      }

      case 8:
        primitives_->do_suction_off();
        break;

      case 9:
        sequences_->move_to_carry_loaded();
        break;

      case 10:
        RCLCPP_INFO(node_->get_logger(), ">>> Move to load preset");
        motion_client_->request_mode_switch("moving");
        primitives_->do_load();
        break;

      case 11:
        if (request_payload_estimate()) {
          RCLCPP_INFO(node_->get_logger(), "Estimated payload mass: %.3f kg", last_estimated_mass_);
        }
        break;

      case 12:
        RCLCPP_INFO(node_->get_logger(), ">>> 3-Phase Grasp");
        three_phase_pipeline_->do_grasp();
        break;

      case 13:
        RCLCPP_INFO(node_->get_logger(), ">>> 3-Phase Place");
        three_phase_pipeline_->do_place();
        break;

      case 14:
        RCLCPP_INFO(node_->get_logger(), ">>> Auto Stack (dog camera perception)");
        if (!sequences_->stack_mock_or_perception()) {
          RCLCPP_ERROR(node_->get_logger(), "[case14] Stack move failed.");
          break;
        }
        RCLCPP_INFO(node_->get_logger(), "[case14] Auto Stack done.");
        break;

      case 15:
      {
        RCLCPP_INFO(node_->get_logger(), ">>> Manual Stack (input box top pose)");
        double tx = 0.0;
        double ty = 0.0;
        double tz = 0.0;
        double tyaw = 0.0;
        std::cout << "Enter box top surface center x y z (world, m) and yaw (rad): ";
        if (!(std::cin >> tx >> ty >> tz >> tyaw)) {
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          RCLCPP_WARN(node_->get_logger(), "Invalid input.");
          break;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        geometry_msgs::msg::Pose box_top_pose;
        box_top_pose.position.x = tx;
        box_top_pose.position.y = ty;
        box_top_pose.position.z = tz;
        box_top_pose.orientation.z = std::sin(tyaw / 2.0);
        box_top_pose.orientation.w = std::cos(tyaw / 2.0);

        RCLCPP_INFO(
          node_->get_logger(), "[case15] box_top pos=(%.3f,%.3f,%.3f) yaw=%.3f",
          tx, ty, tz, tyaw);
        if (!sequences_->stack_pose(box_top_pose)) {
          RCLCPP_ERROR(node_->get_logger(), "[case15] Stack move failed.");
          break;
        }
        RCLCPP_INFO(node_->get_logger(), "[case15] Manual Stack done.");
        break;
      }

      case 16:
        RCLCPP_INFO(node_->get_logger(), ">>> Enable payload model");
        request_payload_state(true);
        break;

      case 17:
        RCLCPP_INFO(node_->get_logger(), ">>> Clear payload model");
        request_payload_state(false);
        break;

      case 0:
        RCLCPP_INFO(node_->get_logger(), "Exit.");
        is_running_->store(false);
        rclcpp::shutdown();
        return;

      default:
        RCLCPP_WARN(node_->get_logger(), "Unknown command: %d", input_cmd);
        break;
    }

    const auto cmd_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - cmd_start_time).count();
    std::cout << "\033[1;33m[Timing]\033[0m cmd " << input_cmd
              << " took " << std::fixed << std::setprecision(3)
              << cmd_elapsed << " s\n"
              << std::flush;
  }
}

}  // namespace arm2_task::task
