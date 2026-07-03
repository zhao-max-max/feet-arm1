#pragma once

#include <atomic>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "arm2_task/task/end_effector_client.hpp"
#include "arm2_task/task/motion_client.hpp"
#include "arm2_task/task/task_primitives.hpp"
#include "arm2_task/task/task_sequences.hpp"
#include "arm2_task/task/three_phase_pipeline.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm2_task::task
{

class TerminalTaskInterface
{
public:
  struct Config
  {
    bool require_payload_service{false};
    bool payload_default_has_load{true};
    double payload_default_mass{0.5};
    std::vector<double> payload_default_com{0.0, 0.0, 0.2219};
  };

  TerminalTaskInterface(
    rclcpp::Node * node,
    MotionClient * motion_client,
    EndEffectorClient * end_effector_client,
    TaskPrimitives * primitives,
    TaskSequences * sequences,
    ThreePhasePipeline * three_phase_pipeline,
    const std::map<std::string, Eigen::VectorXd> * presets,
    std::atomic<bool> * is_running,
    Config config);

  void run();

private:
  bool wait_for_user_command(int * input_cmd);
  int request_payload_estimate();
  int request_payload_state(bool has_load);

  rclcpp::Node * node_{nullptr};
  MotionClient * motion_client_{nullptr};
  EndEffectorClient * end_effector_client_{nullptr};
  TaskPrimitives * primitives_{nullptr};
  TaskSequences * sequences_{nullptr};
  ThreePhasePipeline * three_phase_pipeline_{nullptr};
  const std::map<std::string, Eigen::VectorXd> * presets_{nullptr};
  std::atomic<bool> * is_running_{nullptr};
  Config config_;
  double last_estimated_mass_{0.0};
};

}  // namespace arm2_task::task
