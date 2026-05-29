#ifndef INTENT_HYBRID_RUNTIME_CPP__COLLISION_CHECKER_HPP_
#define INTENT_HYBRID_RUNTIME_CPP__COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "intent_hybrid_runtime_cpp/planner_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace planning_scene_monitor {
class PlanningSceneMonitor;
using PlanningSceneMonitorPtr = std::shared_ptr<PlanningSceneMonitor>;
}  // namespace planning_scene_monitor

namespace intent_hybrid_runtime_cpp {

class CollisionChecker {
 public:
  CollisionChecker() = default;

  bool ensureInitialized(
      const rclcpp::Node::SharedPtr &node,
      const std::string &robot_description_param,
      bool use_planning_scene_monitor,
      std::string &error_message);

  bool available() const { return initialized_; }

  bool isStateValid(
      const std::string &group_name,
      const std::vector<std::string> &joint_names,
      const std::vector<double> &q,
      std::string &error_message,
      uint32_t *collision_queries = nullptr) const;

  bool isEdgeValid(
      const std::string &group_name,
      const std::vector<std::string> &joint_names,
      const std::vector<double> &q1,
      const std::vector<double> &q2,
      double edge_resolution,
      std::string &error_message,
      uint32_t *collision_queries = nullptr) const;

  MotionCheckResult checkMotionBatch(
      const std::string &group_name,
      const std::vector<std::string> &joint_names,
      const std::vector<std::vector<double>> &states,
      bool check_edges,
      double edge_resolution) const;

 private:
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  bool initialized_{false};
  bool init_attempted_{false};
  std::string init_error_;
};

}  // namespace intent_hybrid_runtime_cpp

#endif  // INTENT_HYBRID_RUNTIME_CPP__COLLISION_CHECKER_HPP_
