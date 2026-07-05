#include "intent_hybrid_runtime_cpp/ompl_rrt_connect.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"

namespace intent_hybrid_runtime_cpp {

namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;

double distance_l2(const std::vector<double> &a, const std::vector<double> &b) {
  double acc = 0.0;
  const std::size_t n = std::min(a.size(), b.size());
  for (std::size_t i = 0; i < n; ++i) {
    const double d = a[i] - b[i];
    acc += d * d;
  }
  return std::sqrt(acc);
}

std::vector<double> state_to_vector(const ob::State *state, std::size_t dof) {
  std::vector<double> q(dof, 0.0);
  const auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
  for (std::size_t i = 0; i < dof; ++i) {
    q[i] = rv->values[i];
  }
  return q;
}

std::vector<double> make_times(
    const std::vector<std::vector<double>> &path,
    double t_start,
    double t_end) {
  std::vector<double> times(path.size(), t_start);
  if (path.size() <= 1U) {
    return times;
  }
  std::vector<double> acc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    acc[i] = acc[i - 1U] + distance_l2(path[i - 1U], path[i]);
  }
  const double total = std::max(acc.back(), 1e-9);
  const double duration = std::max(t_end - t_start, 1e-6);
  for (std::size_t i = 0; i < path.size(); ++i) {
    times[i] = t_start + duration * (acc[i] / total);
  }
  return times;
}

double path_length(const std::vector<std::vector<double>> &path) {
  double out = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    out += distance_l2(path[i - 1U], path[i]);
  }
  return out;
}

class MoveItMotionValidator : public ob::MotionValidator {
 public:
  MoveItMotionValidator(
      const ob::SpaceInformationPtr &si,
      std::size_t dof,
      double edge_resolution,
      OmplRRTConnect::EdgeValidityFn edge_valid)
      : ob::MotionValidator(si),
        dof_(dof),
        edge_resolution_(edge_resolution),
        edge_valid_(std::move(edge_valid)) {}

  bool checkMotion(const ob::State *s1, const ob::State *s2) const override {
    std::string err;
    const bool ok = edge_valid_(
        state_to_vector(s1, dof_),
        state_to_vector(s2, dof_),
        edge_resolution_,
        err);
    if (ok) {
      ++valid_;
    } else {
      ++invalid_;
    }
    return ok;
  }

  bool checkMotion(
      const ob::State *s1,
      const ob::State *s2,
      std::pair<ob::State *, double> &last_valid) const override {
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
  std::size_t dof_{0U};
  double edge_resolution_{0.02};
  OmplRRTConnect::EdgeValidityFn edge_valid_;
};

bool validate_path(
    std::vector<std::vector<double>> &path,
    const RRTConnectRequestData &req,
    const OmplRRTConnect::StateValidityFn &state_valid,
    const OmplRRTConnect::EdgeValidityFn &edge_valid,
    std::string &err) {
  if (path.size() < 2U) {
    err = "OMPL path has fewer than 2 points";
    return false;
  }
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (path[i].size() != req.dof) {
      err = "OMPL path point dimension mismatch";
      return false;
    }
  }

  const double endpoint_tol = std::max(req.goal_tolerance, 1e-6);
  if (distance_l2(path.front(), req.start) > endpoint_tol) {
    err = "OMPL path does not start at requested start";
    return false;
  }
  if (distance_l2(path.back(), req.goal) > endpoint_tol) {
    err = "OMPL path does not end near requested goal";
    return false;
  }

  path.front() = req.start;
  if (distance_l2(path.back(), req.goal) > 1e-9) {
    if (!edge_valid(path.back(), req.goal, req.edge_resolution, err)) {
      err = "OMPL final edge to exact goal invalid: " + err;
      return false;
    }
    path.push_back(req.goal);
  } else {
    path.back() = req.goal;
  }

  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!state_valid(path[i], err)) {
      std::ostringstream oss;
      oss << "OMPL path state " << i << " invalid: " << err;
      err = oss.str();
      return false;
    }
  }
  for (std::size_t i = 0; i + 1U < path.size(); ++i) {
    if (!edge_valid(path[i], path[i + 1U], req.edge_resolution, err)) {
      std::ostringstream oss;
      oss << "OMPL path edge " << i << " invalid: " << err;
      err = oss.str();
      return false;
    }
  }
  return true;
}

}  // namespace

OmplRRTConnect::OmplRRTConnect(OmplRRTConnectOptions options)
    : options_(std::move(options)) {}

RRTConnectResult OmplRRTConnect::plan(
    const RRTConnectRequestData &req,
    const StateValidityFn &state_valid,
    const EdgeValidityFn &edge_valid) const {
  RRTConnectResult res;
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
  };
  auto mark_failure = [&](const std::string &reason, const std::string &message = std::string()) {
    res.ok = false;
    res.stop_reason = reason;
    res.error_message = message;
    res.path.clear();
    res.via_times.clear();
    res.elapsed_ms = elapsed_ms();
    return res;
  };

  if (req.dof == 0U || req.start.size() != req.dof || req.goal.size() != req.dof ||
      req.joint_names.size() != req.dof) {
    return mark_failure("invalid_request", "dimension mismatch in OMPL local planner request");
  }
  if (req.state_min.size() != req.dof || req.state_max.size() != req.dof) {
    return mark_failure("invalid_request", "state bounds size mismatch");
  }
  for (std::size_t i = 0; i < req.dof; ++i) {
    if (!(req.state_min[i] < req.state_max[i])) {
      return mark_failure("invalid_request", "invalid state bounds");
    }
  }

  std::string err;
  if (!state_valid(req.start, err)) {
    return mark_failure("collision_start", err);
  }
  if (!state_valid(req.goal, err)) {
    return mark_failure("collision_goal", err);
  }

  auto space = std::make_shared<ob::RealVectorStateSpace>(static_cast<unsigned int>(req.dof));
  ob::RealVectorBounds bounds(static_cast<unsigned int>(req.dof));
  for (std::size_t i = 0; i < req.dof; ++i) {
    bounds.setLow(static_cast<unsigned int>(i), req.state_min[i]);
    bounds.setHigh(static_cast<unsigned int>(i), req.state_max[i]);
  }
  space->setBounds(bounds);

  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker([&](const ob::State *state) {
    std::string state_err;
    return state_valid(state_to_vector(state, req.dof), state_err);
  });
  si->setMotionValidator(std::make_shared<MoveItMotionValidator>(
      si,
      req.dof,
      req.edge_resolution,
      edge_valid));
  si->setup();

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  for (std::size_t i = 0; i < req.dof; ++i) {
    start[static_cast<unsigned int>(i)] = req.start[i];
    goal[static_cast<unsigned int>(i)] = req.goal[i];
  }

  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(start, goal, std::max(req.goal_tolerance, 1e-6));

  auto planner = std::make_shared<og::RRTConnect>(si);
  planner->setRange(std::max(req.step_size, 1e-6));
  planner->setProblemDefinition(pdef);
  planner->setup();

  const double timeout_sec = std::max(req.timeout_sec, 1e-6);
  const auto status = planner->solve(ob::timedPlannerTerminationCondition(timeout_sec));
  ob::PlannerData pdata(si);
  planner->getPlannerData(pdata);
  res.iter_used = static_cast<uint32_t>(
      std::min<std::size_t>(pdata.numVertices(), std::numeric_limits<uint32_t>::max()));

  if (status != ob::PlannerStatus::EXACT_SOLUTION) {
    std::ostringstream oss;
    oss << "OMPL solve status=" << status.asString()
        << ", planner_vertices=" << pdata.numVertices();
    return mark_failure(status ? "approximate_solution" : "timeout", oss.str());
  }

  auto path_base = pdef->getSolutionPath();
  auto path_geo = std::dynamic_pointer_cast<og::PathGeometric>(path_base);
  if (!path_geo) {
    return mark_failure("invalid_solution", "OMPL did not return a geometric path");
  }

  const std::size_t raw_state_count = path_geo->getStateCount();
  const double raw_length = path_geo->length();
  bool simplify_changed = false;
  double simplify_ms = 0.0;
  if (options_.simplify_enable) {
    const auto simplify_t0 = std::chrono::steady_clock::now();
    og::PathSimplifier simplifier(si);
    const double simplify_timeout_sec = std::max(options_.simplify_timeout_sec, 0.0);
    const bool simplify_ok = simplifier.simplify(
        *path_geo,
        simplify_timeout_sec,
        options_.simplify_at_least_once);
    simplify_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - simplify_t0).count();
    if (!simplify_ok || !path_geo->check()) {
      std::ostringstream oss;
      oss << "OMPL PathSimplifier produced invalid path"
          << ", simplify_timeout_sec=" << simplify_timeout_sec
          << ", simplify_ms=" << simplify_ms;
      return mark_failure("invalid_solution", oss.str());
    }
    simplify_changed = path_geo->getStateCount() != raw_state_count ||
                       std::abs(path_geo->length() - raw_length) > 1e-9;
  }

  std::vector<std::vector<double>> path;
  path.reserve(path_geo->getStateCount());
  for (const auto *state : path_geo->getStates()) {
    path.push_back(state_to_vector(state, req.dof));
  }
  if (!validate_path(path, req, state_valid, edge_valid, err)) {
    return mark_failure("invalid_solution", err);
  }

  res.ok = true;
  res.path = std::move(path);
  res.via_times = make_times(res.path, req.t_start, req.t_end);
  res.stop_reason = "ompl_exact";
  std::ostringstream detail;
  detail << "planner=" << (options_.simplify_enable ? "ompl_rrt_connect_simplify" : "ompl_rrt_connect_raw")
         << ", planner_vertices=" << pdata.numVertices()
         << ", raw_path_points=" << raw_state_count
         << ", raw_path_length=" << raw_length
         << ", path_points=" << res.path.size()
         << ", path_length=" << path_length(res.path)
         << ", simplify_enable=" << (options_.simplify_enable ? "true" : "false")
         << ", simplify_changed=" << (simplify_changed ? "true" : "false")
         << ", simplify_timeout_sec=" << std::max(options_.simplify_timeout_sec, 0.0)
         << ", simplify_ms=" << simplify_ms;
  res.error_message = detail.str();
  res.elapsed_ms = elapsed_ms();
  return res;
}

}  // namespace intent_hybrid_runtime_cpp
