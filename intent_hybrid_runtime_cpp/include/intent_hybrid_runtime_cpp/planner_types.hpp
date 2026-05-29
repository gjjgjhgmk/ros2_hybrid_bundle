#ifndef INTENT_HYBRID_RUNTIME_CPP__PLANNER_TYPES_HPP_
#define INTENT_HYBRID_RUNTIME_CPP__PLANNER_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace intent_hybrid_runtime_cpp {

struct MotionCheckResult {
  bool ok{false};
  std::vector<bool> state_valid;
  std::vector<bool> edge_valid;
  int32_t first_invalid_state{-1};
  int32_t first_invalid_edge{-1};
  std::string error_message;
  double elapsed_ms{0.0};
  uint32_t collision_queries{0};
};

struct RRTConnectRequestData {
  std::string group_name;
  std::vector<std::string> joint_names;
  std::size_t dof{0};
  std::vector<double> start;
  std::vector<double> goal;
  std::vector<std::vector<double>> intent_path;
  double t_start{0.0};
  double t_end{0.0};
  std::vector<double> state_min;
  std::vector<double> state_max;
  double timeout_sec{0.1};
  uint32_t max_iter{500};
  double step_size{0.15};
  double goal_tolerance{0.08};
  double edge_resolution{0.02};
  double p_intent{0.45};
  double p_goal{0.20};
  double p_uniform{0.35};
  double sigma_intent{0.08};
  uint32_t rng_seed{42};
};

struct RRTConnectResult {
  bool ok{false};
  std::vector<std::vector<double>> path;
  std::vector<double> via_times;
  std::string stop_reason;
  std::string error_message;
  double elapsed_ms{0.0};
  uint32_t iter_used{0};
  uint32_t collision_queries{0};
};

}  // namespace intent_hybrid_runtime_cpp

#endif  // INTENT_HYBRID_RUNTIME_CPP__PLANNER_TYPES_HPP_
