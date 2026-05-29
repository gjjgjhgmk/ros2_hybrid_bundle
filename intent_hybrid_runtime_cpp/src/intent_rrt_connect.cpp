#include "intent_hybrid_runtime_cpp/intent_rrt_connect.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>

namespace intent_hybrid_runtime_cpp {

namespace {

struct NodeData {
  std::vector<double> q;
  int parent{-1};
};

struct Tree {
  std::vector<NodeData> nodes;
};

double distance_l2(const std::vector<double> &a, const std::vector<double> &b) {
  double acc = 0.0;
  const std::size_t n = std::min(a.size(), b.size());
  for (std::size_t i = 0; i < n; ++i) {
    const double d = a[i] - b[i];
    acc += d * d;
  }
  return std::sqrt(acc);
}

std::vector<double> steer(const std::vector<double> &from, const std::vector<double> &to, double step_size) {
  std::vector<double> out = from;
  const double dist = distance_l2(from, to);
  if (dist <= std::max(step_size, 1e-9)) {
    return to;
  }
  const double a = std::max(step_size, 1e-9) / std::max(dist, 1e-9);
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = from[i] + a * (to[i] - from[i]);
  }
  return out;
}

int nearest_index(const Tree &tree, const std::vector<double> &q) {
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
    const double d = distance_l2(tree.nodes[i].q, q);
    if (d < best_d) {
      best_d = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

std::vector<std::vector<double>> trace_to_root(const Tree &tree, int idx) {
  std::vector<std::vector<double>> path;
  while (idx >= 0 && static_cast<std::size_t>(idx) < tree.nodes.size()) {
    path.push_back(tree.nodes[static_cast<std::size_t>(idx)].q);
    idx = tree.nodes[static_cast<std::size_t>(idx)].parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<double> sample_uniform(
    const std::vector<double> &lo,
    const std::vector<double> &hi,
    std::mt19937 &rng) {
  std::vector<double> q(lo.size(), 0.0);
  for (std::size_t i = 0; i < q.size(); ++i) {
    std::uniform_real_distribution<double> dist(lo[i], hi[i]);
    q[i] = dist(rng);
  }
  return q;
}

std::vector<double> sample_intent(
    const RRTConnectRequestData &req,
    std::mt19937 &rng) {
  if (req.intent_path.empty()) {
    return sample_uniform(req.state_min, req.state_max, rng);
  }
  std::uniform_int_distribution<std::size_t> idx_dist(0U, req.intent_path.size() - 1U);
  std::normal_distribution<double> noise(0.0, std::max(req.sigma_intent, 1e-9));
  std::vector<double> q = req.intent_path[idx_dist(rng)];
  for (std::size_t i = 0; i < q.size(); ++i) {
    q[i] += noise(rng);
    if (i < req.state_min.size() && i < req.state_max.size()) {
      q[i] = std::min(std::max(q[i], req.state_min[i]), req.state_max[i]);
    }
  }
  return q;
}

bool extend_once(
    Tree &tree,
    const std::vector<double> &target,
    const RRTConnectRequestData &req,
    const IntentRRTConnect::StateValidityFn &state_valid,
    const IntentRRTConnect::EdgeValidityFn &edge_valid,
    std::string &err,
    int &new_idx) {
  new_idx = -1;
  const int near_idx = nearest_index(tree, target);
  if (near_idx < 0) {
    err = "nearest failed";
    return false;
  }
  const auto q_new = steer(tree.nodes[static_cast<std::size_t>(near_idx)].q, target, req.step_size);
  if (!state_valid(q_new, err)) {
    return false;
  }
  if (!edge_valid(tree.nodes[static_cast<std::size_t>(near_idx)].q, q_new, req.edge_resolution, err)) {
    return false;
  }
  tree.nodes.push_back(NodeData{q_new, near_idx});
  new_idx = static_cast<int>(tree.nodes.size() - 1U);
  return true;
}

bool connect_tree(
    Tree &tree,
    const std::vector<double> &target,
    const RRTConnectRequestData &req,
    const IntentRRTConnect::StateValidityFn &state_valid,
    const IntentRRTConnect::EdgeValidityFn &edge_valid,
    std::string &err,
    int &last_idx) {
  last_idx = -1;
  while (true) {
    int idx = -1;
    if (!extend_once(tree, target, req, state_valid, edge_valid, err, idx)) {
      return false;
    }
    last_idx = idx;
    if (distance_l2(tree.nodes[static_cast<std::size_t>(idx)].q, target) <= req.goal_tolerance) {
      return true;
    }
  }
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

}  // namespace

RRTConnectResult IntentRRTConnect::plan(
    const RRTConnectRequestData &req,
    const StateValidityFn &state_valid,
    const EdgeValidityFn &edge_valid) const {
  RRTConnectResult res;
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  };

  if (req.dof == 0U || req.start.size() != req.dof || req.goal.size() != req.dof ||
      req.joint_names.size() != req.dof) {
    res.stop_reason = "invalid_request";
    res.error_message = "dimension mismatch in local planner request";
    res.elapsed_ms = elapsed_ms();
    return res;
  }
  if (req.state_min.size() != req.dof || req.state_max.size() != req.dof) {
    res.stop_reason = "invalid_request";
    res.error_message = "state bounds size mismatch";
    res.elapsed_ms = elapsed_ms();
    return res;
  }

  std::string err;
  if (!state_valid(req.start, err)) {
    res.stop_reason = "collision_start";
    res.error_message = err;
    res.elapsed_ms = elapsed_ms();
    return res;
  }
  if (!state_valid(req.goal, err)) {
    res.stop_reason = "collision_goal";
    res.error_message = err;
    res.elapsed_ms = elapsed_ms();
    return res;
  }

  Tree start_tree;
  Tree goal_tree;
  start_tree.nodes.push_back(NodeData{req.start, -1});
  goal_tree.nodes.push_back(NodeData{req.goal, -1});

  std::mt19937 rng(req.rng_seed);
  const double p_intent = std::max(req.p_intent, 0.0);
  const double p_goal = std::max(req.p_goal, 0.0);
  const double p_uniform = std::max(req.p_uniform, 0.0);
  const double p_sum = std::max(p_intent + p_goal + p_uniform, 1e-9);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  const double timeout_ms = req.timeout_sec > 0.0 ? req.timeout_sec * 1000.0 : 0.0;
  const uint32_t max_iter = std::max<uint32_t>(req.max_iter, 1U);

  for (uint32_t iter = 0; iter < max_iter; ++iter) {
    res.iter_used = iter + 1U;
    if (timeout_ms > 0.0 && elapsed_ms() >= timeout_ms) {
      res.stop_reason = "timeout";
      res.elapsed_ms = elapsed_ms();
      return res;
    }

    const double r = unit(rng);
    std::vector<double> q_rand;
    if (r < p_goal / p_sum) {
      q_rand = req.goal;
    } else if (r < (p_goal + p_intent) / p_sum) {
      q_rand = sample_intent(req, rng);
    } else {
      q_rand = sample_uniform(req.state_min, req.state_max, rng);
    }

    Tree *ta = (iter % 2U == 0U) ? &start_tree : &goal_tree;
    Tree *tb = (iter % 2U == 0U) ? &goal_tree : &start_tree;
    int a_idx = -1;
    if (!extend_once(*ta, q_rand, req, state_valid, edge_valid, err, a_idx)) {
      continue;
    }
    int b_idx = -1;
    const bool connected = connect_tree(*tb, ta->nodes[static_cast<std::size_t>(a_idx)].q, req, state_valid, edge_valid, err, b_idx);
    if (!connected) {
      continue;
    }

    std::vector<std::vector<double>> from_start;
    std::vector<std::vector<double>> from_goal;
    if (iter % 2U == 0U) {
      from_start = trace_to_root(start_tree, a_idx);
      from_goal = trace_to_root(goal_tree, b_idx);
    } else {
      from_start = trace_to_root(start_tree, b_idx);
      from_goal = trace_to_root(goal_tree, a_idx);
    }
    std::reverse(from_goal.begin(), from_goal.end());
    if (!from_start.empty() && !from_goal.empty() && distance_l2(from_start.back(), from_goal.front()) < 1e-9) {
      from_goal.erase(from_goal.begin());
    }
    res.path = from_start;
    res.path.insert(res.path.end(), from_goal.begin(), from_goal.end());
    res.via_times = make_times(res.path, req.t_start, req.t_end);
    res.ok = true;
    res.stop_reason = "success";
    res.elapsed_ms = elapsed_ms();
    return res;
  }

  res.stop_reason = "max_iter";
  res.elapsed_ms = elapsed_ms();
  return res;
}

}  // namespace intent_hybrid_runtime_cpp
