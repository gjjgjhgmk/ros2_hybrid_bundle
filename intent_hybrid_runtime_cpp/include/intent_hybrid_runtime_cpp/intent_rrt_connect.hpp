#ifndef INTENT_HYBRID_RUNTIME_CPP__INTENT_RRT_CONNECT_HPP_
#define INTENT_HYBRID_RUNTIME_CPP__INTENT_RRT_CONNECT_HPP_

#include <functional>
#include <string>
#include <vector>

#include "intent_hybrid_runtime_cpp/planner_types.hpp"

namespace intent_hybrid_runtime_cpp {

class IntentRRTConnect {
 public:
  using StateValidityFn = std::function<bool(const std::vector<double> &, std::string &)>;
  using EdgeValidityFn = std::function<bool(const std::vector<double> &, const std::vector<double> &, double, std::string &)>;

  RRTConnectResult plan(
      const RRTConnectRequestData &req,
      const StateValidityFn &state_valid,
      const EdgeValidityFn &edge_valid) const;
};

}  // namespace intent_hybrid_runtime_cpp

#endif  // INTENT_HYBRID_RUNTIME_CPP__INTENT_RRT_CONNECT_HPP_
