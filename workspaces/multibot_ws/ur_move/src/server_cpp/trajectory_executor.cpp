#include "ur_move/trajectory_executor.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <algorithm>

namespace ur_move {

TrajectoryExecutor::TrajectoryExecutor(rclcpp::Node::SharedPtr node)
    : node_(node), executor_running_(false), goal_accepted_(false) {
    startExecutor();
}

TrajectoryExecutor::~TrajectoryExecutor() {
    stopExecutor();
}

void TrajectoryExecutor::startExecutor() {
    if (executor_running_) {
        return;
    }
    
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_running_ = true;
    
    executor_thread_ = std::thread([this]() {
        while (executor_running_ && rclcpp::ok()) {
            executor_->spin_some(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void TrajectoryExecutor::stopExecutor() {
    if (!executor_running_) {
        return;
    }
    
    executor_running_ = false;
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    executor_.reset();
}

std::string TrajectoryExecutor::getActionName(const std::string& group_name) {
    if (group_name == "left_arm") {
        return "left_arm_controller/follow_joint_trajectory";
    } else if (group_name == "right_arm") {
        return "right_arm_controller/follow_joint_trajectory";
    }
    return group_name + "_controller/follow_joint_trajectory";
}

std::vector<std::string> TrajectoryExecutor::getJointNames(const std::string& group_name) {
    if (group_name == "left_arm") {
        return {
            "left_shoulder_pan_joint",
            "left_shoulder_lift_joint",
            "left_elbow_joint",
            "left_wrist_1_joint",
            "left_wrist_2_joint",
            "left_wrist_3_joint"
        };
    } else if (group_name == "right_arm") {
        return {
            "right_shoulder_pan_joint",
            "right_shoulder_lift_joint",
            "right_elbow_joint",
            "right_wrist_1_joint",
            "right_wrist_2_joint",
            "right_wrist_3_joint"
        };
    }
    return {};
}

bool TrajectoryExecutor::isActionServerAvailable(const std::string& group_name) {
    std::string action_name = getActionName(group_name);
    
    auto it = action_clients_.find(group_name);
    if (it == action_clients_.end()) {
        action_clients_[group_name] = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node_, action_name);
        it = action_clients_.find(group_name);
    }
    
    return it->second->wait_for_action_server(std::chrono::seconds(1));
}

std::chrono::seconds TrajectoryExecutor::calculateTimeout(size_t num_points) const {
    // 动态计算超时时间：基础5秒 + 每点50ms，最大30秒
    const size_t base_timeout_sec = 5;
    const size_t per_point_timeout_ms = 50;
    const size_t max_timeout_sec = 30;
    
    size_t calculated_timeout_sec = base_timeout_sec + (num_points * per_point_timeout_ms) / 1000;
    size_t timeout_sec = std::min(calculated_timeout_sec, max_timeout_sec);
    
    return std::chrono::seconds(timeout_sec);
}

trajectory_msgs::msg::JointTrajectory TrajectoryExecutor::sanitizeTrajectoryTiming(
    const trajectory_msgs::msg::JointTrajectory& trajectory) const {
    auto sanitized = trajectory;
    constexpr int64_t kMinStepNanoseconds = 1000000;  // 1 ms
    int64_t previous_nanoseconds = -1;
    size_t adjusted_points = 0;

    for (auto& point : sanitized.points) {
        int64_t current_nanoseconds =
            static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
            static_cast<int64_t>(point.time_from_start.nanosec);

        int64_t min_allowed_nanoseconds = previous_nanoseconds + kMinStepNanoseconds;
        if (current_nanoseconds < min_allowed_nanoseconds) {
            current_nanoseconds = min_allowed_nanoseconds;
            point.time_from_start.sec = static_cast<int32_t>(current_nanoseconds / 1000000000LL);
            point.time_from_start.nanosec = static_cast<uint32_t>(current_nanoseconds % 1000000000LL);
            adjusted_points++;
        }

        previous_nanoseconds = current_nanoseconds;
    }

    if (adjusted_points > 0) {
        RCLCPP_WARN(node_->get_logger(),
                    "Adjusted %zu trajectory point timestamps to be strictly increasing",
                    adjusted_points);
    }

    return sanitized;
}

bool TrajectoryExecutor::executeTrajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& group_name,
    bool wait_for_completion) {
    
    if (trajectory.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Empty trajectory");
        return false;
    }
    
    // 获取或创建 action client（每个组一个 client）
    std::string action_name = getActionName(group_name);
    auto it = action_clients_.find(group_name);
    if (it == action_clients_.end()) {
        action_clients_[group_name] = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node_, action_name);
        it = action_clients_.find(group_name);
    }
    
    auto action_client = it->second;
    // 等待 action server 可用（最多 5 秒）
    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available: %s", action_name.c_str());
        return false;
    }
    
    // 重置状态并准备 goal 消息
    goal_accepted_ = false;
    accepted_goal_handle_.reset();
    
    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory = sanitizeTrajectoryTiming(trajectory.joint_trajectory);
    
    size_t num_points = trajectory.joint_trajectory.points.size();
    auto timeout = calculateTimeout(num_points);
    
    // 设置回调函数
    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &TrajectoryExecutor::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
        &TrajectoryExecutor::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &TrajectoryExecutor::resultCallback, this, std::placeholders::_1);
    
    auto future = action_client->async_send_goal(goal_msg, send_goal_options);
    if (!wait_for_completion) {
        return true;
    }
    
    // 等待 goal 被接受：使用双重检测机制（future + callback）
    auto start_time = std::chrono::steady_clock::now();
    const int max_retries = 5;
    int retry_count = 0;
    
    while (true) {
        // 主要机制：检查 future 状态
        if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
            break;
    }
    
        // 备用机制：检查回调是否已触发
        if (goal_accepted_) {
            break;
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > timeout) {
            if (retry_count < max_retries) {
                // 指数退避重试：500ms, 1s, 2s, 4s, 8s
                int retry_wait_ms = 500 * (1 << retry_count);
                RCLCPP_WARN(node_->get_logger(), 
                           "Goal acceptance timeout (%ld ms, %zu points), retry %d/%d...", 
                           std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
                           num_points, retry_count + 1, max_retries);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_wait_ms));
                retry_count++;
                
                // 重试后检查
                if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready || goal_accepted_) {
                    break;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), 
                           "Goal acceptance failed after %d retries (%zu points, %ld ms)", 
                           max_retries, num_points, 
                           std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
                return false;
            }
        }
    }
    
    // 获取 goal_handle：优先使用 future，失败则使用回调保存的 handle
    std::shared_ptr<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle;
    
    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
            goal_handle = future.get();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get goal handle: %s", e.what());
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            goal_handle = accepted_goal_handle_;
        }
    } else if (goal_accepted_) {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        goal_handle = accepted_goal_handle_;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Cannot get goal handle");
        return false;
    }
    
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
        return false;
    }
    
    // 等待执行完成
    auto result_future = action_client->async_get_result(goal_handle);
    // 使用固定的大超时值（600秒=10分钟），以应对示教器速度设置很慢的情况
    auto result_timeout = std::chrono::seconds(600);
    auto result_start_time = std::chrono::steady_clock::now();
    
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        auto elapsed = std::chrono::steady_clock::now() - result_start_time;
        if (elapsed > result_timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory execution timeout (%ld ms)", 
                        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
        return false;
        }
    }
    
    // 检查执行结果
    auto result = result_future.get();
    
    std::string code_str;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            code_str = "SUCCEEDED";
            break;
        case rclcpp_action::ResultCode::ABORTED:
            code_str = "ABORTED";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            code_str = "CANCELED";
            break;
        default:
            code_str = "UNKNOWN(" + std::to_string(static_cast<int>(result.code)) + ")";
            break;
    }
    
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Trajectory execution failed: %s", code_str.c_str());
        if (result.result && !result.result->error_string.empty()) {
            RCLCPP_WARN(node_->get_logger(), "Error: %s", result.result->error_string.c_str());
        }
        return false;
    }
}

void TrajectoryExecutor::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle) {
    if (!goal_handle) {
        goal_accepted_ = false;
        RCLCPP_WARN(node_->get_logger(), "Goal was rejected");
    } else {
        // 保存 goal_handle 作为备用机制（当 future 状态更新延迟时使用）
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        goal_accepted_ = true;
        accepted_goal_handle_ = goal_handle;
    }
}

void TrajectoryExecutor::feedbackCallback(
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {
    (void)goal_handle;
    (void)feedback;
}

void TrajectoryExecutor::resultCallback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result) {
    // 结果回调（用于异步通知，主要逻辑在 executeTrajectory 中处理）
    (void)result;
}

} // namespace ur_move
