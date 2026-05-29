#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_tolerance.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "intent_hybrid_interfaces/srv/check_motion_batch.hpp"
#include "intent_hybrid_interfaces/srv/check_states_batch.hpp"
#include "intent_hybrid_interfaces/srv/dispatch_joint_trajectory.hpp"
#include "intent_hybrid_interfaces/srv/plan_local_segment.hpp"
#include "intent_hybrid_interfaces/srv/publish_planning_markers.hpp"
#include "intent_hybrid_runtime_cpp/collision_checker.hpp"
#include "intent_hybrid_runtime_cpp/intent_rrt_connect.hpp"
#include "intent_hybrid_runtime_cpp/planner_types.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class IntentRuntimeBridge : public rclcpp::Node {
 public:
  using CheckStatesBatch = intent_hybrid_interfaces::srv::CheckStatesBatch;
  using CheckMotionBatch = intent_hybrid_interfaces::srv::CheckMotionBatch;
  using DispatchJointTrajectory = intent_hybrid_interfaces::srv::DispatchJointTrajectory;
  using PlanLocalSegment = intent_hybrid_interfaces::srv::PlanLocalSegment;
  using PublishPlanningMarkers = intent_hybrid_interfaces::srv::PublishPlanningMarkers;
  using GetStateValidity = moveit_msgs::srv::GetStateValidity;
  using GetPositionFK = moveit_msgs::srv::GetPositionFK;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  IntentRuntimeBridge() : Node("intent_runtime_bridge") {
    moveit_group_name_ = this->declare_parameter<std::string>("moveit_group_name", "ur_manipulator");
    use_planning_scene_monitor_ = this->declare_parameter<bool>("use_planning_scene_monitor", true);
    robot_description_param_ = this->declare_parameter<std::string>("robot_description_param", "robot_description");
    default_edge_resolution_ = this->declare_parameter<double>("default_edge_resolution", 0.02);
    default_planner_timeout_sec_ = this->declare_parameter<double>("default_planner_timeout_sec", 0.10);
    default_planner_max_iter_ = this->declare_parameter<int>("default_planner_max_iter", 500);
    default_planner_step_size_ = this->declare_parameter<double>("default_planner_step_size", 0.15);
    default_goal_tolerance_ = this->declare_parameter<double>("default_goal_tolerance", 0.08);
    state_stale_timeout_sec_ = this->declare_parameter<double>("state_stale_timeout_sec", 1.0);
    state_validity_service_wait_sec_ = this->declare_parameter<double>("state_validity_service_wait_sec", 3.0);
    state_validity_call_timeout_sec_ = this->declare_parameter<double>("state_validity_call_timeout_sec", 1.0);
    fk_service_wait_sec_ = this->declare_parameter<double>("fk_service_wait_sec", 3.0);
    fk_timeout_sec_ = this->declare_parameter<double>("fk_timeout_sec", 0.5);
    action_server_wait_sec_ = this->declare_parameter<double>("action_server_wait_sec", 3.0);
    dispatch_result_wait_sec_ = this->declare_parameter<double>("dispatch_result_wait_sec", 30.0);

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    state_validity_client_ =
        this->create_client<GetStateValidity>("/check_state_validity", rmw_qos_profile_services_default, client_cb_group_);
    fk_client_ = this->create_client<GetPositionFK>("/compute_fk", rmw_qos_profile_services_default, client_cb_group_);

    auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning_vis", marker_qos);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = sub_cb_group_;
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(20),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(joint_state_mutex_);
          latest_joint_state_ = msg;
          latest_joint_state_time_ = this->now();
          has_joint_state_ = true;
        },
        sub_opts);

    check_states_srv_ = this->create_service<CheckStatesBatch>(
        "/intent_runtime/check_states_batch",
        std::bind(&IntentRuntimeBridge::handle_check_states, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    check_motion_srv_ = this->create_service<CheckMotionBatch>(
        "/intent_runtime/check_motion_batch",
        std::bind(&IntentRuntimeBridge::handle_check_motion, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    plan_local_segment_srv_ = this->create_service<PlanLocalSegment>(
        "/intent_runtime/plan_local_segment",
        std::bind(&IntentRuntimeBridge::handle_plan_local_segment, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    dispatch_srv_ = this->create_service<DispatchJointTrajectory>(
        "/intent_runtime/dispatch_joint_trajectory",
        std::bind(&IntentRuntimeBridge::handle_dispatch_trajectory, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    publish_markers_srv_ = this->create_service<PublishPlanningMarkers>(
        "/intent_runtime/publish_planning_markers",
        std::bind(&IntentRuntimeBridge::handle_publish_markers, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    RCLCPP_INFO(
        this->get_logger(),
        "intent_runtime_bridge started (state_stale_timeout=%.2fs, state_validity_wait=%.2fs, "
        "state_validity_call_timeout=%.2fs, fk_wait=%.2fs, fk_timeout=%.2fs, action_wait=%.2fs, "
        "dispatch_result_wait=%.2fs, moveit_group=%s, psm=%s).",
        state_stale_timeout_sec_,
        state_validity_service_wait_sec_,
        state_validity_call_timeout_sec_,
        fk_service_wait_sec_,
        fk_timeout_sec_,
        action_server_wait_sec_,
        dispatch_result_wait_sec_,
        moveit_group_name_.c_str(),
        use_planning_scene_monitor_ ? "true" : "false");
  }

 private:
  struct MatrixRows {
    std::vector<std::vector<double>> rows;
    size_t dof{0};
  };

  template <typename ServiceT>
  bool wait_for_service_ready(
      const typename rclcpp::Client<ServiceT>::SharedPtr &client,
      double total_wait_sec) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(std::max(total_wait_sec, 0.05)));
    while (std::chrono::steady_clock::now() < deadline) {
      if (client->wait_for_service(200ms)) {
        return true;
      }
    }
    return false;
  }

  bool parse_flat_rows(const std::vector<double> &flat, size_t dof, MatrixRows &out, std::string &err) const {
    if (dof == 0U) {
      err = "dof must be > 0";
      return false;
    }
    if (flat.size() % dof != 0U) {
      std::ostringstream oss;
      oss << "flat size " << flat.size() << " is not divisible by dof " << dof;
      err = oss.str();
      return false;
    }
    const size_t n = flat.size() / dof;
    out.rows.assign(n, std::vector<double>(dof, 0.0));
    out.dof = dof;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < dof; ++j) {
        out.rows[i][j] = flat[i * dof + j];
      }
    }
    return true;
  }

  bool get_ordered_joint_positions(const std::vector<std::string> &joint_names, std::vector<double> &q_out) {
    std::lock_guard<std::mutex> lk(joint_state_mutex_);
    if (!has_joint_state_ || !latest_joint_state_) {
      return false;
    }
    const double age = (this->now() - latest_joint_state_time_).seconds();
    if (age > state_stale_timeout_sec_) {
      return false;
    }
    if (latest_joint_state_->name.empty() || latest_joint_state_->position.empty()) {
      return false;
    }

    std::map<std::string, double> m;
    const auto &names = latest_joint_state_->name;
    const auto &pos = latest_joint_state_->position;
    for (size_t i = 0; i < names.size() && i < pos.size(); ++i) {
      m[names[i]] = pos[i];
    }

    q_out.assign(joint_names.size(), 0.0);
    for (size_t i = 0; i < joint_names.size(); ++i) {
      const auto it = m.find(joint_names[i]);
      if (it == m.end()) {
        return false;
      }
      q_out[i] = it->second;
    }
    return true;
  }

  bool call_state_validity_once(
      const std::string &group_name,
      const std::vector<std::string> &joint_names,
      const std::vector<double> &q,
      bool &collision_free,
      std::string &err) {
    if (!wait_for_service_ready<GetStateValidity>(state_validity_client_, state_validity_service_wait_sec_)) {
      err = "/check_state_validity not ready";
      return false;
    }

    auto req = std::make_shared<GetStateValidity::Request>();
    req->group_name = group_name.empty() ? moveit_group_name_ : group_name;
    req->robot_state = moveit_msgs::msg::RobotState();
    req->robot_state.joint_state.name = joint_names;
    req->robot_state.joint_state.position = q;

    auto fut = state_validity_client_->async_send_request(req);
    const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(state_validity_call_timeout_sec_, 0.05)));
    if (fut.wait_for(timeout) != std::future_status::ready) {
      err = "check_state_validity timeout";
      return false;
    }
    auto resp = fut.get();
    if (!resp) {
      err = "check_state_validity returned null";
      return false;
    }
    collision_free = bool(resp->valid);
    return true;
  }

  bool fk_once(
      const std::string &base_frame,
      const std::string &ee_link,
      const std::vector<std::string> &joint_names,
      const std::vector<double> &q,
      geometry_msgs::msg::Point &p_out,
      std::string &err) {
    if (!wait_for_service_ready<GetPositionFK>(fk_client_, fk_service_wait_sec_)) {
      err = "/compute_fk not ready";
      return false;
    }

    auto req = std::make_shared<GetPositionFK::Request>();
    req->header.frame_id = base_frame;
    req->fk_link_names = {ee_link.empty() ? std::string("tool0") : ee_link};
    req->robot_state = moveit_msgs::msg::RobotState();
    req->robot_state.joint_state.name = joint_names;
    req->robot_state.joint_state.position = q;

    auto fut = fk_client_->async_send_request(req);
    const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(fk_timeout_sec_, 0.05)));
    if (fut.wait_for(timeout) != std::future_status::ready) {
      err = "compute_fk timeout";
      return false;
    }
    auto resp = fut.get();
    if (!resp) {
      err = "compute_fk returned null";
      return false;
    }
    if (resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || resp->pose_stamped.empty()) {
      std::ostringstream oss;
      oss << "compute_fk failed, code=" << resp->error_code.val;
      err = oss.str();
      return false;
    }
    p_out = resp->pose_stamped[0].pose.position;
    return true;
  }

  void handle_check_states(
      const std::shared_ptr<CheckStatesBatch::Request> req,
      std::shared_ptr<CheckStatesBatch::Response> res) {
    res->ok = false;
    res->collision_free.clear();
    res->error_message.clear();

    MatrixRows mat;
    std::string err;
    if (!parse_flat_rows(req->states_flat, static_cast<size_t>(req->dof), mat, err)) {
      res->error_message = err;
      return;
    }
    if (req->joint_names.size() != mat.dof) {
      res->error_message = "joint_names size mismatch with dof";
      return;
    }

    std::string init_err;
    if (collision_checker_.ensureInitialized(
            shared_from_this(),
            robot_description_param_,
            use_planning_scene_monitor_,
            init_err)) {
      const auto checked = collision_checker_.checkMotionBatch(
          req->group_name.empty() ? moveit_group_name_ : req->group_name,
          req->joint_names,
          mat.rows,
          false,
          default_edge_resolution_);
      if (checked.ok && checked.state_valid.size() == mat.rows.size()) {
        res->collision_free = checked.state_valid;
        res->ok = true;
        return;
      }
      RCLCPP_WARN(
          this->get_logger(),
          "PlanningScene check_states failed; fallback to /check_state_validity: %s",
          checked.error_message.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "PlanningSceneMonitor unavailable for check_states; fallback to /check_state_validity: %s",
          init_err.c_str());
    }

    res->collision_free.reserve(mat.rows.size());
    for (const auto &row : mat.rows) {
      bool ok = false;
      bool valid = false;
      ok = call_state_validity_once(req->group_name, req->joint_names, row, valid, err);
      if (!ok) {
        res->error_message = err;
        return;
      }
      res->collision_free.push_back(valid);
    }

    res->ok = true;
  }

  void handle_check_motion(
      const std::shared_ptr<CheckMotionBatch::Request> req,
      std::shared_ptr<CheckMotionBatch::Response> res) {
    res->ok = false;
    res->state_valid.clear();
    res->edge_valid.clear();
    res->first_invalid_state = -1;
    res->first_invalid_edge = -1;
    res->error_message.clear();
    res->elapsed_ms = 0.0;
    res->collision_queries = 0U;

    MatrixRows mat;
    std::string err;
    if (!parse_flat_rows(req->states_flat, static_cast<size_t>(req->dof), mat, err)) {
      res->error_message = err;
      return;
    }
    if (req->joint_names.size() != mat.dof) {
      res->error_message = "joint_names size mismatch with dof";
      return;
    }
    if (mat.rows.empty()) {
      res->ok = true;
      return;
    }

    std::string init_err;
    if (!collision_checker_.ensureInitialized(
            shared_from_this(),
            robot_description_param_,
            use_planning_scene_monitor_,
            init_err)) {
      res->error_message = init_err.empty() ? "PlanningSceneMonitor unavailable" : init_err;
      return;
    }

    const auto checked = collision_checker_.checkMotionBatch(
        req->group_name.empty() ? moveit_group_name_ : req->group_name,
        req->joint_names,
        mat.rows,
        req->check_edges,
        req->edge_resolution > 0.0 ? req->edge_resolution : default_edge_resolution_);
    res->ok = checked.ok;
    res->state_valid = checked.state_valid;
    res->edge_valid = checked.edge_valid;
    res->first_invalid_state = checked.first_invalid_state;
    res->first_invalid_edge = checked.first_invalid_edge;
    res->error_message = checked.error_message;
    res->elapsed_ms = checked.elapsed_ms;
    res->collision_queries = checked.collision_queries;
  }

  bool parse_intent_rows(
      const std::vector<double> &flat,
      size_t points,
      size_t dof,
      std::vector<std::vector<double>> &out,
      std::string &err) const {
    out.clear();
    if (points == 0U || flat.empty()) {
      return true;
    }
    if (flat.size() != points * dof) {
      std::ostringstream oss;
      oss << "intent_flat size " << flat.size() << " != intent_points*dof " << (points * dof);
      err = oss.str();
      return false;
    }
    out.assign(points, std::vector<double>(dof, 0.0));
    for (size_t i = 0; i < points; ++i) {
      for (size_t j = 0; j < dof; ++j) {
        out[i][j] = flat[i * dof + j];
      }
    }
    return true;
  }

  void handle_plan_local_segment(
      const std::shared_ptr<PlanLocalSegment::Request> req,
      std::shared_ptr<PlanLocalSegment::Response> res) {
    res->ok = false;
    res->path_flat.clear();
    res->path_points = 0U;
    res->via_times.clear();
    res->stop_reason = "invalid_request";
    res->error_message.clear();
    res->elapsed_ms = 0.0;
    res->iter_used = 0U;
    res->collision_queries = 0U;

    intent_hybrid_runtime_cpp::RRTConnectRequestData plan_req;
    plan_req.group_name = req->group_name.empty() ? moveit_group_name_ : req->group_name;
    plan_req.joint_names = req->joint_names;
    plan_req.dof = static_cast<size_t>(req->dof);
    plan_req.start = req->start;
    plan_req.goal = req->goal;
    plan_req.t_start = req->t_start;
    plan_req.t_end = req->t_end;
    plan_req.state_min = req->state_min;
    plan_req.state_max = req->state_max;
    plan_req.timeout_sec = req->timeout_sec > 0.0 ? req->timeout_sec : default_planner_timeout_sec_;
    plan_req.max_iter = req->max_iter > 0U ? req->max_iter : static_cast<uint32_t>(std::max(default_planner_max_iter_, 1));
    plan_req.step_size = req->step_size > 0.0 ? req->step_size : default_planner_step_size_;
    plan_req.goal_tolerance = req->goal_tolerance > 0.0 ? req->goal_tolerance : default_goal_tolerance_;
    plan_req.edge_resolution = req->edge_resolution > 0.0 ? req->edge_resolution : default_edge_resolution_;
    plan_req.p_intent = req->p_intent;
    plan_req.p_goal = req->p_goal;
    plan_req.p_uniform = req->p_uniform;
    plan_req.sigma_intent = req->sigma_intent;
    plan_req.rng_seed = req->rng_seed;

    std::string err;
    if (!parse_intent_rows(req->intent_flat, static_cast<size_t>(req->intent_points), plan_req.dof, plan_req.intent_path, err)) {
      res->error_message = err;
      return;
    }
    if (plan_req.state_min.empty()) {
      plan_req.state_min.assign(plan_req.dof, -6.283185307179586);
    }
    if (plan_req.state_max.empty()) {
      plan_req.state_max.assign(plan_req.dof, 6.283185307179586);
    }

    std::string init_err;
    if (!collision_checker_.ensureInitialized(
            shared_from_this(),
            robot_description_param_,
            use_planning_scene_monitor_,
            init_err)) {
      res->stop_reason = "invalid_request";
      res->error_message = init_err.empty() ? "PlanningSceneMonitor unavailable" : init_err;
      return;
    }

    uint32_t collision_queries = 0U;
    intent_hybrid_runtime_cpp::IntentRRTConnect planner;
    auto state_valid = [&](const std::vector<double> &q, std::string &e) {
      return collision_checker_.isStateValid(plan_req.group_name, plan_req.joint_names, q, e, &collision_queries);
    };
    auto edge_valid = [&](const std::vector<double> &a, const std::vector<double> &b, double resolution, std::string &e) {
      return collision_checker_.isEdgeValid(plan_req.group_name, plan_req.joint_names, a, b, resolution, e, &collision_queries);
    };
    const auto planned = planner.plan(plan_req, state_valid, edge_valid);

    res->ok = planned.ok;
    res->path_points = static_cast<uint32_t>(planned.path.size());
    res->via_times = planned.via_times;
    res->stop_reason = planned.stop_reason;
    res->error_message = planned.error_message;
    res->elapsed_ms = planned.elapsed_ms;
    res->iter_used = planned.iter_used;
    res->collision_queries = collision_queries;
    if (planned.ok) {
      res->path_flat.reserve(planned.path.size() * plan_req.dof);
      for (const auto &row : planned.path) {
        for (double v : row) {
          res->path_flat.push_back(v);
        }
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "plan_local_segment ok=%s stop=%s iter=%u time=%.2fms queries=%u path_points=%u",
        res->ok ? "true" : "false",
        res->stop_reason.c_str(),
        res->iter_used,
        res->elapsed_ms,
        res->collision_queries,
        res->path_points);
  }

  std::vector<double> build_time_axis(size_t n_points, double nominal_dt) const {
    std::vector<double> t(n_points, 0.0);
    const double dt = std::max(nominal_dt, 1e-3);
    for (size_t i = 0; i < n_points; ++i) {
      t[i] = static_cast<double>(i) * dt;
    }
    return t;
  }

  void compute_vel_acc(
      const std::vector<std::vector<double>> &q,
      const std::vector<double> &t,
      std::vector<std::vector<double>> &v,
      std::vector<std::vector<double>> &a) const {
    const size_t n = q.size();
    const size_t d = q.empty() ? 0U : q[0].size();
    v.assign(n, std::vector<double>(d, 0.0));
    a.assign(n, std::vector<double>(d, 0.0));

    for (size_t k = 1; k < n; ++k) {
      const double dt = std::max(t[k] - t[k - 1], 1e-6);
      for (size_t j = 0; j < d; ++j) {
        v[k][j] = (q[k][j] - q[k - 1][j]) / dt;
      }
    }

    for (size_t k = 1; k < n; ++k) {
      const double dt = std::max(t[k] - t[k - 1], 1e-6);
      for (size_t j = 0; j < d; ++j) {
        a[k][j] = (v[k][j] - v[k - 1][j]) / dt;
      }
    }
  }

  void scale_time_for_limits(
      const std::vector<std::vector<double>> &q,
      std::vector<double> &t,
      const std::vector<double> &vel_limits,
      const std::vector<double> &acc_limits) const {
    if (q.size() < 2 || q[0].empty()) {
      return;
    }
    const size_t d = q[0].size();
    auto limits_v = vel_limits;
    auto limits_a = acc_limits;
    if (limits_v.size() != d) {
      limits_v.assign(d, 1.0);
    }
    if (limits_a.size() != d) {
      limits_a.assign(d, 2.0);
    }

    std::vector<std::vector<double>> v;
    std::vector<std::vector<double>> a;
    compute_vel_acc(q, t, v, a);

    double vel_ratio = 1.0;
    double acc_ratio = 1.0;
    for (size_t k = 0; k < v.size(); ++k) {
      for (size_t j = 0; j < d; ++j) {
        const double vlim = std::max(std::abs(limits_v[j]), 1e-6);
        const double alim = std::max(std::abs(limits_a[j]), 1e-6);
        vel_ratio = std::max(vel_ratio, std::abs(v[k][j]) / vlim);
        acc_ratio = std::max(acc_ratio, std::abs(a[k][j]) / alim);
      }
    }

    const double scale = std::max(1.0, std::max(vel_ratio, acc_ratio));
    if (scale <= 1.0) {
      return;
    }
    for (auto &x : t) {
      x *= scale;
    }
  }

  double compute_jerk_proxy(
      const std::vector<std::vector<double>> &a,
      const std::vector<double> &t) const {
    if (a.size() < 2 || a[0].empty()) {
      return 0.0;
    }
    const size_t d = a[0].size();
    double out = 0.0;
    for (size_t k = 1; k < a.size(); ++k) {
      const double dt = std::max(t[k] - t[k - 1], 1e-6);
      for (size_t j = 0; j < d; ++j) {
        out = std::max(out, std::abs(a[k][j] - a[k - 1][j]) / dt);
      }
    }
    return out;
  }

  trajectory_msgs::msg::JointTrajectory build_trajectory_msg(
      const std::vector<std::string> &joint_names,
      const std::vector<std::vector<double>> &q,
      const std::vector<std::vector<double>> &v,
      const std::vector<std::vector<double>> &a,
      const std::vector<double> &t) const {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names;

    for (size_t k = 0; k < q.size(); ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = q[k];
      pt.velocities = v[k];
      pt.accelerations = a[k];

      const int32_t sec = static_cast<int32_t>(std::floor(t[k]));
      const int32_t nanosec = static_cast<int32_t>(std::llround((t[k] - static_cast<double>(sec)) * 1e9));
      builtin_interfaces::msg::Duration dur;
      dur.sec = sec;
      dur.nanosec = nanosec;
      pt.time_from_start = dur;
      traj.points.push_back(pt);
    }
    return traj;
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr get_action_client(const std::string &action_name) {
    const std::string key = action_name.empty() ? std::string("/joint_trajectory_controller/follow_joint_trajectory") : action_name;
    auto it = action_clients_.find(key);
    if (it != action_clients_.end()) {
      return it->second;
    }
    auto client = rclcpp_action::create_client<FollowJointTrajectory>(this, key);
    action_clients_[key] = client;
    return client;
  }

  void handle_dispatch_trajectory(
      const std::shared_ptr<DispatchJointTrajectory::Request> req,
      std::shared_ptr<DispatchJointTrajectory::Response> res) {
    res->accepted = false;
    res->result_code = "rejected_bad_shape";
    res->jerk_proxy = 0.0;
    res->points_sent = 0U;
    res->error_message.clear();

    MatrixRows mat;
    std::string err;
    if (!parse_flat_rows(req->q_flat, static_cast<size_t>(req->dof), mat, err)) {
      res->error_message = err;
      return;
    }
    if (req->joint_names.size() != mat.dof) {
      res->error_message = "joint_names size mismatch with dof";
      return;
    }

    if (req->stitch_from_current) {
      std::vector<double> q_now;
      if (!get_ordered_joint_positions(req->joint_names, q_now)) {
        res->result_code = "failed_stale_state";
        res->error_message = "no fresh joint_states for stitching";
        return;
      }
      mat.rows.insert(mat.rows.begin(), q_now);
    }

    if (mat.rows.size() < 2) {
      mat.rows.push_back(mat.rows.back());
    }

    double start_delta_max = 0.0;
    if (mat.rows.size() >= 2U) {
      for (size_t j = 0; j < mat.dof; ++j) {
        start_delta_max = std::max(start_delta_max, std::abs(mat.rows[1][j] - mat.rows[0][j]));
      }
    }

    auto t = build_time_axis(mat.rows.size(), req->nominal_dt);
    scale_time_for_limits(mat.rows, t, req->vel_limits, req->acc_limits);
    const double start_dt = t.size() >= 2U ? std::max(t[1] - t[0], 0.0) : 0.0;

    std::vector<std::vector<double>> v;
    std::vector<std::vector<double>> a;
    compute_vel_acc(mat.rows, t, v, a);

    res->jerk_proxy = compute_jerk_proxy(a, t);

    auto traj_msg = build_trajectory_msg(req->joint_names, mat.rows, v, a, t);
    auto action_client = get_action_client(req->action_name);
    const auto action_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(action_server_wait_sec_, 0.05)));
    if (!action_client->wait_for_action_server(action_timeout)) {
      res->result_code = "failed_server_offline";
      res->error_message = "action server not ready";
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj_msg;
    if (req->path_tolerance_rad > 0.0) {
      goal.path_tolerance.clear();
      goal.path_tolerance.reserve(req->joint_names.size());
      for (const auto &name : req->joint_names) {
        control_msgs::msg::JointTolerance tol;
        tol.name = name;
        tol.position = static_cast<double>(req->path_tolerance_rad);
        goal.path_tolerance.push_back(tol);
      }
    }
    if (req->goal_tolerance_rad > 0.0) {
      goal.goal_tolerance.clear();
      goal.goal_tolerance.reserve(req->joint_names.size());
      for (const auto &name : req->joint_names) {
        control_msgs::msg::JointTolerance tol;
        tol.name = name;
        tol.position = static_cast<double>(req->goal_tolerance_rad);
        goal.goal_tolerance.push_back(tol);
      }
    }
    if (req->goal_time_tolerance_sec > 0.0) {
      const auto sec_part = static_cast<int32_t>(std::floor(req->goal_time_tolerance_sec));
      const auto nanosec_part = static_cast<uint32_t>(std::llround(
          (req->goal_time_tolerance_sec - static_cast<double>(sec_part)) * 1e9));
      goal.goal_time_tolerance.sec = sec_part;
      goal.goal_time_tolerance.nanosec = nanosec_part;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "dispatch trajectory summary: points=%zu, start_delta_max=%.4f rad, first_dt=%.4fs, "
        "jerk_proxy=%.3f, path_tol=%.3f, goal_tol=%.3f, goal_time_tol=%.3f",
        traj_msg.points.size(), start_delta_max, start_dt, res->jerk_proxy,
        static_cast<double>(req->path_tolerance_rad),
        static_cast<double>(req->goal_tolerance_rad),
        static_cast<double>(req->goal_time_tolerance_sec));

    auto send_goal_future = action_client->async_send_goal(goal);
    if (send_goal_future.wait_for(action_timeout) != std::future_status::ready) {
      res->result_code = "failed_send_goal_timeout";
      res->error_message = "send_goal timeout";
      return;
    }
    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      res->result_code = "rejected_by_action_server";
      res->error_message = "goal rejected";
      return;
    }

    res->accepted = true;
    res->points_sent = static_cast<uint32_t>(traj_msg.points.size());

    const double expected_exec_sec = t.empty() ? 0.0 : std::max(t.back(), 0.0);
    const double result_wait_sec = std::max(dispatch_result_wait_sec_, expected_exec_sec + 2.0);
    const auto result_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(result_wait_sec, 0.05)));

    auto result_future = action_client->async_get_result(goal_handle);
    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      res->result_code = "failed_action_result_timeout";
      std::ostringstream oss;
      oss << "action result timeout after " << result_wait_sec << " sec";
      res->error_message = oss.str();
      return;
    }

    auto wrapped_result = result_future.get();
    const auto action_code = wrapped_result.code;
    const auto result_msg = wrapped_result.result;
    const int32_t follow_error_code =
        result_msg ? static_cast<int32_t>(result_msg->error_code) : static_cast<int32_t>(-9999);
    const std::string follow_error_string = result_msg ? std::string(result_msg->error_string) : std::string("");

    if (action_code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (follow_error_code == static_cast<int32_t>(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)) {
        res->result_code = "sent_success";
        return;
      }
      res->result_code = "failed_action_error_code";
      std::ostringstream oss;
      oss << "FollowJointTrajectory error_code=" << follow_error_code;
      if (!follow_error_string.empty()) {
        oss << ", msg=" << follow_error_string;
      }
      res->error_message = oss.str();
      return;
    }

    if (action_code == rclcpp_action::ResultCode::ABORTED) {
      res->result_code = "failed_action_aborted";
      std::ostringstream oss;
      oss << "Action aborted (error_code=" << follow_error_code << ")";
      if (!follow_error_string.empty()) {
        oss << ", msg=" << follow_error_string;
      }
      res->error_message = oss.str();
      return;
    }

    if (action_code == rclcpp_action::ResultCode::CANCELED) {
      res->result_code = "failed_action_canceled";
      std::ostringstream oss;
      oss << "Action canceled (error_code=" << follow_error_code << ")";
      if (!follow_error_string.empty()) {
        oss << ", msg=" << follow_error_string;
      }
      res->error_message = oss.str();
      return;
    }

    res->result_code = "failed_action_unknown";
    {
      std::ostringstream oss;
      oss << "Action result unknown code, follow_error_code=" << follow_error_code;
      if (!follow_error_string.empty()) {
        oss << ", msg=" << follow_error_string;
      }
      res->error_message = oss.str();
    }
  }

  bool add_fk_line_points(
      const std::vector<std::vector<double>> &rows,
      const std::string &base_frame,
      const std::string &ee_link,
      const std::vector<std::string> &joint_names,
      std::vector<geometry_msgs::msg::Point> &out,
      std::string &err) {
    out.clear();
    out.reserve(rows.size());
    for (const auto &q : rows) {
      geometry_msgs::msg::Point p;
      if (!fk_once(base_frame, ee_link, joint_names, q, p, err)) {
        return false;
      }
      out.push_back(p);
    }
    return true;
  }

  void handle_publish_markers(
      const std::shared_ptr<PublishPlanningMarkers::Request> req,
      std::shared_ptr<PublishPlanningMarkers::Response> res) {
    res->ok = false;
    res->error_message.clear();

    const size_t dof = static_cast<size_t>(req->dof);
    if (dof == 0U || req->joint_names.size() != dof) {
      res->error_message = "invalid dof/joint_names";
      return;
    }

    MatrixRows nominal;
    MatrixRows via;
    MatrixRows modulated;
    std::string err;
    if (!parse_flat_rows(req->nominal_q_flat, dof, nominal, err)) {
      res->error_message = std::string("nominal_q_flat: ") + err;
      return;
    }
    if (!req->via_q_flat.empty() && !parse_flat_rows(req->via_q_flat, dof, via, err)) {
      res->error_message = std::string("via_q_flat: ") + err;
      return;
    }
    if (!parse_flat_rows(req->modulated_q_flat, dof, modulated, err)) {
      res->error_message = std::string("modulated_q_flat: ") + err;
      return;
    }

    const std::string base_frame = req->base_frame.empty() ? std::string("base_link") : req->base_frame;
    const std::string ee_link = req->ee_link.empty() ? std::string("tool0") : req->ee_link;

    std::vector<geometry_msgs::msg::Point> nominal_pts;
    std::vector<geometry_msgs::msg::Point> via_pts;
    std::vector<geometry_msgs::msg::Point> modulated_pts;

    if (!add_fk_line_points(nominal.rows, base_frame, ee_link, req->joint_names, nominal_pts, err)) {
      res->error_message = std::string("nominal FK failed: ") + err;
      return;
    }
    if (!add_fk_line_points(modulated.rows, base_frame, ee_link, req->joint_names, modulated_pts, err)) {
      res->error_message = std::string("modulated FK failed: ") + err;
      return;
    }
    if (!via.rows.empty() && !add_fk_line_points(via.rows, base_frame, ee_link, req->joint_names, via_pts, err)) {
      res->error_message = std::string("via FK failed: ") + err;
      return;
    }

    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = this->now();

    visualization_msgs::msg::Marker m0;
    m0.header.frame_id = base_frame;
    m0.header.stamp = stamp;
    m0.ns = "planning_vis";
    m0.id = 0;
    m0.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m0.action = visualization_msgs::msg::Marker::ADD;
    m0.pose.orientation.w = 1.0;
    m0.scale.x = 0.01;
    m0.color.r = 0.0;
    m0.color.g = 0.0;
    m0.color.b = 1.0;
    m0.color.a = 0.5;
    m0.points = nominal_pts;

    visualization_msgs::msg::Marker m1;
    m1.header.frame_id = base_frame;
    m1.header.stamp = stamp;
    m1.ns = "planning_vis";
    m1.id = 1;
    m1.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m1.action = visualization_msgs::msg::Marker::ADD;
    m1.pose.orientation.w = 1.0;
    m1.scale.x = 0.06;
    m1.scale.y = 0.06;
    m1.scale.z = 0.06;
    m1.color.r = 1.0;
    m1.color.g = 0.0;
    m1.color.b = 0.0;
    m1.color.a = 1.0;
    m1.points = via_pts;

    visualization_msgs::msg::Marker m2;
    m2.header.frame_id = base_frame;
    m2.header.stamp = stamp;
    m2.ns = "planning_vis";
    m2.id = 2;
    m2.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m2.action = visualization_msgs::msg::Marker::ADD;
    m2.pose.orientation.w = 1.0;
    m2.scale.x = 0.015;
    m2.color.r = 0.0;
    m2.color.g = 1.0;
    m2.color.b = 0.0;
    m2.color.a = 0.8;
    m2.points = modulated_pts;

    arr.markers = {m0, m1, m2};

    if (!req->nominal_ee_xyz_flat.empty() && req->nominal_ee_xyz_flat.size() % 3U == 0U) {
      visualization_msgs::msg::Marker m3;
      m3.header.frame_id = base_frame;
      m3.header.stamp = stamp;
      m3.ns = "planning_vis";
      m3.id = 3;
      m3.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m3.action = visualization_msgs::msg::Marker::ADD;
      m3.pose.orientation.w = 1.0;
      m3.scale.x = 0.01;
      m3.color.r = 1.0;
      m3.color.g = 0.6;
      m3.color.b = 0.0;
      m3.color.a = 0.8;
      for (size_t i = 0; i < req->nominal_ee_xyz_flat.size(); i += 3) {
        geometry_msgs::msg::Point p;
        p.x = req->nominal_ee_xyz_flat[i + 0];
        p.y = req->nominal_ee_xyz_flat[i + 1];
        p.z = req->nominal_ee_xyz_flat[i + 2];
        m3.points.push_back(p);
      }
      arr.markers.push_back(m3);
    }

    if (!req->obstacle_xyzr_flat.empty() && req->obstacle_xyzr_flat.size() % 4U == 0U) {
      visualization_msgs::msg::Marker m4;
      m4.header.frame_id = base_frame;
      m4.header.stamp = stamp;
      m4.ns = "planning_vis";
      m4.id = 4;
      m4.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      m4.action = visualization_msgs::msg::Marker::ADD;
      m4.pose.orientation.w = 1.0;

      double max_r = 0.04;
      for (size_t i = 0; i < req->obstacle_xyzr_flat.size(); i += 4) {
        max_r = std::max(max_r, std::max(req->obstacle_xyzr_flat[i + 3], 1e-3));
      }
      const double d = std::max(2.0 * max_r, 0.02);
      m4.scale.x = d;
      m4.scale.y = d;
      m4.scale.z = d;
      m4.color.r = 1.0;
      m4.color.g = 1.0;
      m4.color.b = 0.0;
      m4.color.a = 0.6;
      for (size_t i = 0; i < req->obstacle_xyzr_flat.size(); i += 4) {
        geometry_msgs::msg::Point p;
        p.x = req->obstacle_xyzr_flat[i + 0];
        p.y = req->obstacle_xyzr_flat[i + 1];
        p.z = req->obstacle_xyzr_flat[i + 2];
        m4.points.push_back(p);
      }
      arr.markers.push_back(m4);
    }

    marker_pub_->publish(arr);
    res->ok = true;
  }

  std::string moveit_group_name_;
  bool use_planning_scene_monitor_{true};
  std::string robot_description_param_{"robot_description"};
  double default_edge_resolution_{0.02};
  double default_planner_timeout_sec_{0.10};
  int default_planner_max_iter_{500};
  double default_planner_step_size_{0.15};
  double default_goal_tolerance_{0.08};
  double state_stale_timeout_sec_{1.0};
  double state_validity_service_wait_sec_{3.0};
  double state_validity_call_timeout_sec_{1.0};
  double fk_service_wait_sec_{3.0};
  double fk_timeout_sec_{0.5};
  double action_server_wait_sec_{3.0};
  double dispatch_result_wait_sec_{30.0};

  rclcpp::Client<GetStateValidity>::SharedPtr state_validity_client_;
  rclcpp::Client<GetPositionFK>::SharedPtr fk_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Service<CheckStatesBatch>::SharedPtr check_states_srv_;
  rclcpp::Service<CheckMotionBatch>::SharedPtr check_motion_srv_;
  rclcpp::Service<DispatchJointTrajectory>::SharedPtr dispatch_srv_;
  rclcpp::Service<PlanLocalSegment>::SharedPtr plan_local_segment_srv_;
  rclcpp::Service<PublishPlanningMarkers>::SharedPtr publish_markers_srv_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  std::map<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> action_clients_;

  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  rclcpp::Time latest_joint_state_time_{0, 0, RCL_ROS_TIME};
  bool has_joint_state_{false};

  intent_hybrid_runtime_cpp::CollisionChecker collision_checker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IntentRuntimeBridge>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
