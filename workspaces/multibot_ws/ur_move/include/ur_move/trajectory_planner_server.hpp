#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <zmq.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "ur_move/waypoint_message.hpp"
#include "ur_move/trajectory_executor.hpp"

namespace ur_move {

class MoveItPlanner;

struct PendingExecution {
    std::map<std::string, moveit_msgs::msg::RobotTrajectory> trajectories;
    std::chrono::high_resolution_clock::time_point timestamp;
};

class TrajectoryPlannerServer {
public:
    explicit TrajectoryPlannerServer(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryPlannerServer();
    
    bool start(int bind_port);
    void stop();
    
    rclcpp::Logger getLogger() const { return node_->get_logger(); }

private:
    void serverLoop();
    void handleRequest(const std::string& request_data, MoveItPlanner& planner);
    void sendResponse(const std::string& response_data);
    void sendErrorResponse(const std::string& error_message);
    void logStatistics();
    
    // Execution ID management
    std::string generateExecutionId();
    std::string storePendingExecution(const std::map<std::string, moveit_msgs::msg::RobotTrajectory>& trajectories);
    bool executePendingTrajectory(const std::string& execution_id);
    
    // JSON to RobotTrajectory conversion
    moveit_msgs::msg::RobotTrajectory jsonToRobotTrajectory(const nlohmann::json& traj_json);
    
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<zmq::context_t> zmq_context_;
    std::unique_ptr<zmq::socket_t> zmq_socket_;
    std::string bind_address_;
    
    std::thread server_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};
    
    // Execution ID storage
    std::map<std::string, PendingExecution> pending_executions_;
    std::mutex execution_mutex_;
    std::atomic<uint64_t> execution_id_counter_{0};
    
    // Statistics
    std::atomic<uint64_t> total_requests_{0};
    std::atomic<uint64_t> successful_requests_{0};
    std::atomic<uint64_t> failed_requests_{0};
};

} // namespace ur_move

