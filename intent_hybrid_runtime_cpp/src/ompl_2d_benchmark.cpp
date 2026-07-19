#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/planners/rrt/InformedRRTstar.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/util/Console.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using json = nlohmann::json;

namespace {

struct CircleObstacle {
  std::array<double, 2> center{0.0, 0.0};
  double radius{0.0};
};

struct Request {
  std::string planner_type{"rrt_connect"};
  std::array<double, 2> start{0.1, 0.5};
  std::array<double, 2> goal{0.9, 0.5};
  std::vector<CircleObstacle> obstacles;
  double timeout_sec{1.0};
  double step_size{0.04};
  double goal_tolerance{0.04};
  double edge_resolution{0.01};
  double goal_bias{0.05};
  std::uint32_t rng_seed{42};
  bool simplify_enable{false};
  double simplify_timeout_sec{0.05};
  bool simplify_at_least_once{true};
};

std::string lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::array<double, 2> parse_pair(const json &value, const char *name) {
  if (!value.is_array() || value.size() != 2 || !value[0].is_number() || !value[1].is_number()) {
    std::ostringstream oss;
    oss << name << " must be a length-2 numeric array";
    throw std::runtime_error(oss.str());
  }
  return {value[0].get<double>(), value[1].get<double>()};
}

Request parse_request(const json &root) {
  Request req;
  req.planner_type = lower_copy(root.value("planner_type", req.planner_type));
  if (root.contains("start")) {
    req.start = parse_pair(root.at("start"), "start");
  }
  if (root.contains("goal")) {
    req.goal = parse_pair(root.at("goal"), "goal");
  }
  if (root.contains("timeout_sec")) {
    req.timeout_sec = root.at("timeout_sec").get<double>();
  }
  if (root.contains("step_size")) {
    req.step_size = root.at("step_size").get<double>();
  }
  if (root.contains("goal_tolerance")) {
    req.goal_tolerance = root.at("goal_tolerance").get<double>();
  }
  if (root.contains("edge_resolution")) {
    req.edge_resolution = root.at("edge_resolution").get<double>();
  }
  if (root.contains("goal_bias")) {
    req.goal_bias = root.at("goal_bias").get<double>();
  }
  if (root.contains("rng_seed")) {
    req.rng_seed = root.at("rng_seed").get<std::uint32_t>();
  }
  if (root.contains("simplify_enable")) {
    req.simplify_enable = root.at("simplify_enable").get<bool>();
  }
  if (root.contains("simplify_timeout_sec")) {
    req.simplify_timeout_sec = root.at("simplify_timeout_sec").get<double>();
  }
  if (root.contains("simplify_at_least_once")) {
    req.simplify_at_least_once = root.at("simplify_at_least_once").get<bool>();
  }
  if (root.contains("obstacles")) {
    const auto &items = root.at("obstacles");
    if (!items.is_array()) {
      throw std::runtime_error("obstacles must be an array");
    }
    req.obstacles.reserve(items.size());
    for (const auto &item : items) {
      CircleObstacle obstacle;
      if (!item.is_object()) {
        throw std::runtime_error("each obstacle must be an object");
      }
      obstacle.center = parse_pair(item.at("center"), "obstacle.center");
      obstacle.radius = item.at("radius").get<double>();
      if (!(obstacle.radius > 0.0) || !std::isfinite(obstacle.radius)) {
        throw std::runtime_error("obstacle radius must be a positive finite value");
      }
      req.obstacles.push_back(obstacle);
    }
  }
  if (!(req.timeout_sec > 0.0) || !(req.step_size > 0.0) || !(req.goal_tolerance >= 0.0) ||
      !(req.edge_resolution > 0.0)) {
    throw std::runtime_error("timeout_sec, step_size, goal_tolerance and edge_resolution must be valid");
  }
  return req;
}

std::array<double, 2> state_to_point(const ob::State *state) {
  const auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
  return {rv->values[0], rv->values[1]};
}

double l2(const std::array<double, 2> &a, const std::array<double, 2> &b) {
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  return std::sqrt(dx * dx + dy * dy);
}

double path_length(const std::vector<std::array<double, 2>> &path) {
  if (path.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    total += l2(path[i - 1], path[i]);
  }
  return total;
}

struct CollisionModel {
  std::vector<CircleObstacle> obstacles;
  mutable std::uint64_t queries{0};

  bool point_valid(const std::array<double, 2> &point) const {
    ++queries;
    if (!(point[0] >= 0.0 && point[0] <= 1.0 && point[1] >= 0.0 && point[1] <= 1.0)) {
      return false;
    }
    for (const auto &obstacle : obstacles) {
      if (l2(point, obstacle.center) <= obstacle.radius) {
        return false;
      }
    }
    return true;
  }

  bool edge_valid(const std::array<double, 2> &first, const std::array<double, 2> &second, double resolution) const {
    const double edge_len = l2(first, second);
    const std::size_t samples = std::max<std::size_t>(2U, static_cast<std::size_t>(std::ceil(edge_len / resolution)));
    for (std::size_t i = 0; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples);
      const std::array<double, 2> p{
          first[0] + ratio * (second[0] - first[0]),
          first[1] + ratio * (second[1] - first[1]),
      };
      if (!point_valid(p)) {
        return false;
      }
    }
    return true;
  }
};

class TwoDMotionValidator : public ob::MotionValidator {
 public:
  TwoDMotionValidator(const ob::SpaceInformationPtr &si, std::shared_ptr<CollisionModel> model, double resolution)
      : ob::MotionValidator(si), model_(std::move(model)), resolution_(resolution) {}

  bool checkMotion(const ob::State *s1, const ob::State *s2) const override {
    return model_->edge_valid(state_to_point(s1), state_to_point(s2), resolution_);
  }

  bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &last_valid) const override {
    if (checkMotion(s1, s2)) {
      return true;
    }
    if (last_valid.first != nullptr) {
      si_->copyState(last_valid.first, s1);
    }
    last_valid.second = 0.0;
    return false;
  }

 private:
  std::shared_ptr<CollisionModel> model_;
  double resolution_{0.01};
};

json make_failure(const Request &req, const std::shared_ptr<CollisionModel> &model, const std::string &stop_reason,
                  const std::string &error_message, double elapsed_ms) {
  json out;
  out["ok"] = false;
  out["planner_type"] = req.planner_type;
  out["simplify_enable"] = req.simplify_enable;
  out["stop_reason"] = stop_reason;
  out["error_message"] = error_message;
  out["path"] = json::array();
  out["path_points"] = 0;
  out["raw_state_count"] = 0;
  out["raw_path_length"] = 0.0;
  out["path_length"] = 0.0;
  out["solve_time_ms"] = elapsed_ms;
  out["simplify_time_ms"] = 0.0;
  out["total_time_ms"] = elapsed_ms;
  out["collision_queries"] = model ? model->queries : 0U;
  out["planner_vertices"] = 0;
  out["simplify_changed"] = false;
  out["request"] = {
      {"start", {req.start[0], req.start[1]}},
      {"goal", {req.goal[0], req.goal[1]}},
      {"timeout_sec", req.timeout_sec},
      {"step_size", req.step_size},
      {"goal_tolerance", req.goal_tolerance},
      {"edge_resolution", req.edge_resolution},
      {"goal_bias", req.goal_bias},
      {"rng_seed", req.rng_seed},
      {"obstacle_count", req.obstacles.size()},
  };
  return out;
}

std::shared_ptr<ob::Planner> make_planner(const std::string &planner_type, const ob::SpaceInformationPtr &si,
                                          const Request &req) {
  if (planner_type == "rrt") {
    auto planner = std::make_shared<og::RRT>(si);
    planner->setRange(req.step_size);
    planner->setGoalBias(req.goal_bias);
    return planner;
  }
  if (planner_type == "rrt_star" || planner_type == "rrtstar") {
    auto planner = std::make_shared<og::RRTstar>(si);
    planner->setRange(req.step_size);
    planner->setGoalBias(req.goal_bias);
    return planner;
  }
  if (planner_type == "informed_rrt_star" || planner_type == "informed_rrtstar") {
    auto planner = std::make_shared<og::InformedRRTstar>(si);
    planner->setRange(req.step_size);
    planner->setGoalBias(req.goal_bias);
    return planner;
  }
  if (planner_type == "rrt_connect" || planner_type == "rrtconnect") {
    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setRange(req.step_size);
    return planner;
  }
  throw std::runtime_error("unsupported planner_type: " + planner_type);
}

std::vector<std::array<double, 2>> extract_path(const og::PathGeometric &path) {
  std::vector<std::array<double, 2>> out;
  out.reserve(path.getStateCount());
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    out.push_back(state_to_point(path.getState(i)));
  }
  return out;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    if (argc != 3 || std::string(argv[1]) != "--request-file") {
      std::cerr << "Usage: ompl_2d_benchmark --request-file <json>" << std::endl;
      return 2;
    }

    std::ifstream in(argv[2]);
    if (!in.is_open()) {
      std::cerr << "Failed to open request file: " << argv[2] << std::endl;
      return 2;
    }

    const auto request_json = json::parse(in);
    const Request req = parse_request(request_json);
    const auto started = std::chrono::steady_clock::now();
    auto elapsed_ms = [&]() {
      return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    };

    auto collision_model = std::make_shared<CollisionModel>();
    collision_model->obstacles = req.obstacles;

    if (!collision_model->point_valid(req.start)) {
      std::cout << make_failure(req, collision_model, "collision_start", "start state is in collision", elapsed_ms())
                << std::endl;
      return 0;
    }
    if (!collision_model->point_valid(req.goal)) {
      std::cout << make_failure(req, collision_model, "collision_goal", "goal state is in collision", elapsed_ms())
                << std::endl;
      return 0;
    }

    auto space = std::make_shared<ob::RealVectorStateSpace>(2U);
    ob::RealVectorBounds bounds(2U);
    bounds.setLow(0U, 0.0);
    bounds.setHigh(0U, 1.0);
    bounds.setLow(1U, 0.0);
    bounds.setHigh(1U, 1.0);
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([collision_model](const ob::State *state) {
      return collision_model->point_valid(state_to_point(state));
    });
    si->setMotionValidator(std::make_shared<TwoDMotionValidator>(si, collision_model, req.edge_resolution));
    si->setup();

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = req.start[0];
    start[1] = req.start[1];
    goal[0] = req.goal[0];
    goal[1] = req.goal[1];

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal, std::max(req.goal_tolerance, 1e-6));
    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(objective);

    auto planner = make_planner(req.planner_type, si, req);
    planner->setProblemDefinition(pdef);
    planner->setup();

    const double timeout_sec = std::max(req.timeout_sec, 1e-6);
    const auto solve_t0 = std::chrono::steady_clock::now();
    const auto status = planner->solve(ob::timedPlannerTerminationCondition(timeout_sec));
    const double solve_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - solve_t0).count();

    ob::PlannerData pdata(si);
    planner->getPlannerData(pdata);

    if (!status || status == ob::PlannerStatus::TIMEOUT || !pdef->hasSolution()) {
      std::ostringstream oss;
      oss << "OMPL solve status=" << status.asString() << ", planner_vertices=" << pdata.numVertices();
      std::cout << make_failure(
          req, collision_model, status ? "approximate_solution" : "timeout", oss.str(), elapsed_ms()) << std::endl;
      return 0;
    }

    auto solution_base = pdef->getSolutionPath();
    auto path_geo = std::dynamic_pointer_cast<og::PathGeometric>(solution_base);
    if (!path_geo) {
      std::cout << make_failure(req, collision_model, "invalid_solution", "solution path is not geometric",
                                elapsed_ms()) << std::endl;
      return 0;
    }

    const std::size_t raw_state_count = path_geo->getStateCount();
    const double raw_length = path_geo->length();
    bool simplify_changed = false;
    double simplify_ms = 0.0;
    if (req.simplify_enable) {
      const auto simplify_t0 = std::chrono::steady_clock::now();
      og::PathSimplifier simplifier(si);
      const bool simplify_ok = simplifier.simplify(
          *path_geo,
          std::max(req.simplify_timeout_sec, 0.0),
          req.simplify_at_least_once);
      simplify_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - simplify_t0).count();
      if (!simplify_ok || !path_geo->check()) {
        std::ostringstream oss;
        oss << "PathSimplifier produced invalid path"
            << ", simplify_timeout_sec=" << req.simplify_timeout_sec
            << ", simplify_ms=" << simplify_ms;
        std::cout << make_failure(req, collision_model, "invalid_solution", oss.str(), elapsed_ms()) << std::endl;
        return 0;
      }
      simplify_changed = path_geo->getStateCount() != raw_state_count ||
                         std::abs(path_geo->length() - raw_length) > 1e-9;
    }

    const auto path = extract_path(*path_geo);
    for (const auto &point : path) {
      if (!collision_model->point_valid(point)) {
        std::ostringstream oss;
        oss << "validated path contains a colliding state at [" << point[0] << ", " << point[1] << "]";
        std::cout << make_failure(req, collision_model, "invalid_solution", oss.str(), elapsed_ms()) << std::endl;
        return 0;
      }
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (!collision_model->edge_valid(path[i - 1], path[i], req.edge_resolution)) {
        std::ostringstream oss;
        oss << "validated path contains a colliding edge at segment " << (i - 1);
        std::cout << make_failure(req, collision_model, "invalid_solution", oss.str(), elapsed_ms()) << std::endl;
        return 0;
      }
    }

    json out;
    out["ok"] = true;
    out["planner_type"] = req.planner_type;
    out["simplify_enable"] = req.simplify_enable;
    out["stop_reason"] = "ompl_exact";
    out["error_message"] = status.asString();
    out["path"] = json::array();
    for (const auto &point : path) {
      out["path"].push_back({point[0], point[1]});
    }
    out["path_points"] = path.size();
    out["raw_state_count"] = raw_state_count;
    out["raw_path_length"] = raw_length;
    out["path_length"] = path_length(path);
    out["solve_time_ms"] = solve_ms;
    out["simplify_time_ms"] = simplify_ms;
    out["total_time_ms"] = elapsed_ms();
    out["collision_queries"] = collision_model->queries;
    out["planner_vertices"] = pdata.numVertices();
    out["simplify_changed"] = simplify_changed;
    out["request"] = {
        {"start", {req.start[0], req.start[1]}},
        {"goal", {req.goal[0], req.goal[1]}},
        {"timeout_sec", req.timeout_sec},
        {"step_size", req.step_size},
        {"goal_tolerance", req.goal_tolerance},
        {"edge_resolution", req.edge_resolution},
        {"goal_bias", req.goal_bias},
        {"rng_seed", req.rng_seed},
        {"obstacle_count", req.obstacles.size()},
        {"simplify_timeout_sec", req.simplify_timeout_sec},
        {"simplify_at_least_once", req.simplify_at_least_once},
    };

    std::cout << out.dump(2) << std::endl;
    return 0;
  } catch (const std::exception &exc) {
    json out;
    out["ok"] = false;
    out["planner_type"] = "unknown";
    out["simplify_enable"] = false;
    out["stop_reason"] = "exception";
    out["error_message"] = exc.what();
    out["path"] = json::array();
    out["path_points"] = 0;
    out["raw_state_count"] = 0;
    out["raw_path_length"] = 0.0;
    out["path_length"] = 0.0;
    out["solve_time_ms"] = 0.0;
    out["simplify_time_ms"] = 0.0;
    out["total_time_ms"] = 0.0;
    out["collision_queries"] = 0;
    out["planner_vertices"] = 0;
    out["simplify_changed"] = false;
    out["request"] = json::object();
    std::cout << out.dump(2) << std::endl;
    return 1;
  }
}
