#include "intent_hybrid_runtime_cpp/collision_checker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "moveit/planning_scene/planning_scene.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_model/joint_model_group.h"
#include "moveit/robot_state/robot_state.h"

namespace intent_hybrid_runtime_cpp {

namespace {

std::size_t edge_steps(
    const std::vector<double> &q1,
    const std::vector<double> &q2,
    double edge_resolution) {
  double max_delta = 0.0;
  const std::size_t n = std::min(q1.size(), q2.size());
  for (std::size_t i = 0; i < n; ++i) {
    max_delta = std::max(max_delta, std::abs(q2[i] - q1[i]));
  }
  const double res = std::max(std::abs(edge_resolution), 1e-6);
  return std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(max_delta / res)));
}

}  // namespace

bool CollisionChecker::ensureInitialized(
    const rclcpp::Node::SharedPtr &node,
    const std::string &robot_description_param,
    bool use_planning_scene_monitor,
    std::string &error_message) {
  if (initialized_ && psm_) {
    return true;
  }
  if (!use_planning_scene_monitor) {
    error_message = "planning_scene_monitor disabled";
    return false;
  }
  if (init_attempted_ && !initialized_) {
    error_message = init_error_;
    return false;
  }
  init_attempted_ = true;
  try {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node,
        robot_description_param.empty() ? std::string("robot_description") : robot_description_param);
    if (!psm_ || !psm_->getPlanningScene()) {
      init_error_ = "PlanningSceneMonitor has no planning scene";
      error_message = init_error_;
      return false;
    }
    psm_->startSceneMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->startStateMonitor();
    initialized_ = true;
    error_message.clear();
    RCLCPP_INFO(node->get_logger(), "CollisionChecker initialized with PlanningSceneMonitor.");
    return true;
  } catch (const std::exception &exc) {
    init_error_ = std::string("PlanningSceneMonitor init exception: ") + exc.what();
  } catch (...) {
    init_error_ = "PlanningSceneMonitor init unknown exception";
  }
  error_message = init_error_;
  return false;
}

bool CollisionChecker::isStateValid(
    const std::string &group_name,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &q,
    std::string &error_message,
    uint32_t *collision_queries) const {
  if (collision_queries) {
    ++(*collision_queries);
  }
  if (!initialized_ || !psm_) {
    error_message = init_error_.empty() ? "PlanningSceneMonitor not initialized" : init_error_;
    return false;
  }
  if (joint_names.size() != q.size()) {
    error_message = "joint_names size mismatch with q";
    return false;
  }
  planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
  if (!scene) {
    error_message = "failed to lock planning scene";
    return false;
  }
  const auto *jmg = scene->getRobotModel()->getJointModelGroup(group_name);
  if (jmg == nullptr) {
    error_message = "joint model group not found: " + group_name;
    return false;
  }
  moveit::core::RobotState state(scene->getCurrentState());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const auto *jm = state.getRobotModel()->getJointModel(joint_names[i]);
    if (jm == nullptr) {
      error_message = "joint not found in robot model: " + joint_names[i];
      return false;
    }
    state.setJointPositions(jm, &q[i]);
  }
  state.update();
  const bool colliding = scene->isStateColliding(state, group_name);
  error_message.clear();
  return !colliding;
}

bool CollisionChecker::isEdgeValid(
    const std::string &group_name,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &q1,
    const std::vector<double> &q2,
    double edge_resolution,
    std::string &error_message,
    uint32_t *collision_queries) const {
  if (q1.size() != q2.size()) {
    error_message = "edge endpoints have different dimensions";
    return false;
  }
  const std::size_t steps = edge_steps(q1, q2, edge_resolution);
  std::vector<double> q(q1.size(), 0.0);
  for (std::size_t s = 0; s <= steps; ++s) {
    const double a = static_cast<double>(s) / static_cast<double>(steps);
    for (std::size_t j = 0; j < q.size(); ++j) {
      q[j] = (1.0 - a) * q1[j] + a * q2[j];
    }
    if (!isStateValid(group_name, joint_names, q, error_message, collision_queries)) {
      return false;
    }
  }
  error_message.clear();
  return true;
}

MotionCheckResult CollisionChecker::checkMotionBatch(
    const std::string &group_name,
    const std::vector<std::string> &joint_names,
    const std::vector<std::vector<double>> &states,
    bool check_edges,
    double edge_resolution) const {
  MotionCheckResult out;
  const auto t0 = std::chrono::steady_clock::now();
  out.state_valid.assign(states.size(), false);
  if (check_edges && states.size() >= 2U) {
    out.edge_valid.assign(states.size() - 1U, false);
  }

  std::string err;
  for (std::size_t i = 0; i < states.size(); ++i) {
    const bool valid = isStateValid(group_name, joint_names, states[i], err, &out.collision_queries);
    out.state_valid[i] = valid;
    if (!valid && out.first_invalid_state < 0) {
      out.first_invalid_state = static_cast<int32_t>(i);
    }
  }

  if (check_edges && states.size() >= 2U) {
    for (std::size_t i = 0; i + 1U < states.size(); ++i) {
      const bool valid = isEdgeValid(
          group_name,
          joint_names,
          states[i],
          states[i + 1U],
          edge_resolution,
          err,
          &out.collision_queries);
      out.edge_valid[i] = valid;
      if (!valid && out.first_invalid_edge < 0) {
        out.first_invalid_edge = static_cast<int32_t>(i);
      }
    }
  }

  out.ok = true;
  if (out.first_invalid_state >= 0 || out.first_invalid_edge >= 0) {
    out.error_message.clear();
  } else {
    out.error_message.clear();
  }
  const auto t1 = std::chrono::steady_clock::now();
  out.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return out;
}

}  // namespace intent_hybrid_runtime_cpp
