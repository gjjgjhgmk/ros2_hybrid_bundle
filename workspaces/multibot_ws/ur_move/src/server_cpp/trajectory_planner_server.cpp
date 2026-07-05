#include "ur_move/trajectory_planner_server.hpp"
#include "ur_move/moveit_planner.hpp"
#include <chrono>
#include <nlohmann/json.hpp>
#include <cstring>
#include <set>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace ur_move {

TrajectoryPlannerServer::TrajectoryPlannerServer(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
    zmq_context_ = std::make_unique<zmq::context_t>(1);
    RCLCPP_INFO(getLogger(), "Trajectory Planner Server initialized");
}

TrajectoryPlannerServer::~TrajectoryPlannerServer() {
    stop();
    RCLCPP_INFO(getLogger(), "Trajectory Planner Server destroyed");
}

bool TrajectoryPlannerServer::start(const int bind_port) {
    if (running_) {
        RCLCPP_WARN(getLogger(), "Server is already running");
        return true;
    }
    
    // 创建 ZMQ REP socket（请求-响应模式）
    bind_address_ = "tcp://*:" + std::to_string(bind_port);
    
    try {
        zmq_socket_ = std::make_unique<zmq::socket_t>(*zmq_context_, ZMQ_REP);
        zmq_socket_->set(zmq::sockopt::rcvtimeo, 1000);  // 接收超时 1 秒
        zmq_socket_->set(zmq::sockopt::sndtimeo, 5000);  // 发送超时 5 秒
        zmq_socket_->bind(bind_address_);
        
        RCLCPP_INFO(getLogger(), "ZMQ socket bound to: %s", bind_address_.c_str());
        
        should_stop_ = false;
        running_ = true;
        // 启动服务器循环线程
        server_thread_ = std::thread(&TrajectoryPlannerServer::serverLoop, this);
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Failed to start server: %s", e.what());
        running_ = false;
        return false;
    }
}

void TrajectoryPlannerServer::stop() {
    if (!running_) {
        return;
    }
    
    RCLCPP_INFO(getLogger(), "Stopping Trajectory Planner Server...");
    should_stop_ = true;
    
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    
    if (zmq_socket_) {
        zmq_socket_->close();
        zmq_socket_.reset();
    }
    
    running_ = false;
    logStatistics();
    RCLCPP_INFO(getLogger(), "Trajectory Planner Server stopped");
}

void TrajectoryPlannerServer::serverLoop() {
    RCLCPP_INFO(getLogger(), "Server loop started");
    // 初始化 MoveIt 规划器
    MoveItPlanner planner(node_);
    
    if (!planner.initialize()) {
        RCLCPP_ERROR(getLogger(), "Failed to initialize planner");
        return;
    }
    
    // ZMQ 服务器主循环：接收请求并处理
    while (!should_stop_ && rclcpp::ok()) {
        try {
            zmq::message_t request;
            // 接收客户端请求（非阻塞，超时返回 false）
            if (!zmq_socket_->recv(request, zmq::recv_flags::none)) {
                continue;  // 超时，继续等待
            }
            
            total_requests_++;
            std::string request_data(static_cast<char*>(request.data()), request.size());
            handleRequest(request_data, planner);
            
        } catch (const zmq::error_t& e) {
            if (e.num() == ETERM) {
                break;  // 上下文终止，退出循环
            } else if (e.num() != EAGAIN) {
                RCLCPP_ERROR(getLogger(), "ZMQ error: %s", e.what());
                sendErrorResponse("Internal server error: " + std::string(e.what()));
                failed_requests_++;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(getLogger(), "Exception: %s", e.what());
            sendErrorResponse("Internal server error: " + std::string(e.what()));
            failed_requests_++;
        }
    }
    
    RCLCPP_INFO(getLogger(), "Server loop ended");
}

void TrajectoryPlannerServer::handleRequest(
    const std::string& request_data,
    MoveItPlanner& planner) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        nlohmann::json request_json = nlohmann::json::parse(request_data);
        
        // 处理执行请求：通过 execution_id 执行已规划的轨迹
        if (request_json.contains("action") && request_json["action"] == "execute") {
            if (!request_json.contains("execution_id")) {
                sendErrorResponse("Missing execution_id in execute request");
                failed_requests_++;
                return;
            }
            
            std::string execution_id = request_json["execution_id"].get<std::string>();
            bool success = executePendingTrajectory(execution_id);
            
            nlohmann::json response;
            if (success) {
                response["success"] = true;
                response["message"] = "Trajectory execution completed";
                successful_requests_++;
            } else {
                response["success"] = false;
                response["error"] = "Trajectory execution failed";
                failed_requests_++;
            }
            
            sendResponse(response.dump());
            
            // 计算执行耗时并记录日志
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            if (success) {
                RCLCPP_INFO(getLogger(), "Trajectory execution completed in %ld ms (execution_id: %s)", 
                           duration.count(), execution_id.c_str());
            } else {
                RCLCPP_ERROR(getLogger(), "Trajectory execution failed in %ld ms (execution_id: %s)", 
                            duration.count(), execution_id.c_str());
            }
            return;
        }
        
        // 处理规划请求：解析路径点并规划轨迹
        bool should_execute = request_json.value("execute", true);
        
        WaypointMessage waypoint_message = WaypointMessage::deserialize(request_data);
        
        // 收集所有请求的组（用于验证所有组都成功规划）
        std::set<std::string> requested_groups;
        for (const auto& waypoint : waypoint_message.getWaypoints()) {
            requested_groups.insert(waypoint.getGroup());
        }
        
        // 按组规划轨迹（支持多组同时规划）
        auto trajectories_by_group = planner.planTrajectoriesByGroup(waypoint_message.getWaypoints());
        
        if (trajectories_by_group.empty()) {
            sendErrorResponse("Trajectory planning failed: no valid trajectories generated");
            failed_requests_++;
            return;
        }
        
        // 验证所有请求的组是否都成功规划
        for (const auto& group_name : requested_groups) {
            if (trajectories_by_group.find(group_name) == trajectories_by_group.end()) {
                std::string error_msg = "Trajectory planning failed: group '" + group_name + "' planning failed";
                RCLCPP_ERROR(getLogger(), "%s", error_msg.c_str());
                sendErrorResponse(error_msg);
                failed_requests_++;
                return;
            }
        }
        
        // 构建 JSON 响应：将轨迹数据转换为 JSON 格式
        nlohmann::json response;
        response["success"] = true;
        response["trajectories"] = nlohmann::json::object();
        
        for (const auto& [group_name, trajectory] : trajectories_by_group) {
            nlohmann::json traj_json;
            traj_json["joint_names"] = trajectory.joint_trajectory.joint_names;
            traj_json["points"] = nlohmann::json::array();
            
            // 转换每个轨迹点：位置、速度、加速度、时间戳
            for (const auto& point : trajectory.joint_trajectory.points) {
                nlohmann::json point_json;
                point_json["positions"] = point.positions;
                point_json["velocities"] = point.velocities;
                point_json["accelerations"] = point.accelerations;
                point_json["time_from_start"] = {
                    {"sec", point.time_from_start.sec},
                    {"nanosec", point.time_from_start.nanosec}
                };
                traj_json["points"].push_back(point_json);
            }
            response["trajectories"][group_name] = traj_json;
        }
        
        // 如果不需要立即执行，返回 execution_id（用于延迟执行）
        if (!should_execute) {
            std::string execution_id = storePendingExecution(trajectories_by_group);
            response["execution_id"] = execution_id;
            response["message"] = "Trajectory planning completed, waiting for execution";
        } else {
            // 立即执行轨迹：为每个组创建独立线程并行执行
            bool all_success = true;
            std::vector<std::thread> execution_threads;
            std::map<std::string, bool> execution_results;
            std::mutex results_mutex;
            
            for (const auto& [group_name, trajectory] : trajectories_by_group) {
                execution_threads.emplace_back([this, &results_mutex, &execution_results, &all_success, group_name, trajectory]() {
                    try {
                        // 为每个线程创建独立的节点，避免 ROS 2 executor 冲突
                        auto executor_node = rclcpp::Node::make_shared(
                            "trajectory_executor_" + group_name + "_" + 
                            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())
                        );
                        
                        TrajectoryExecutor executor(executor_node);
                        bool success = executor.executeTrajectory(trajectory, group_name, true);
                        
                        // 记录执行结果（线程安全）
                        std::lock_guard<std::mutex> lock(results_mutex);
                        execution_results[group_name] = success;
                        if (!success) {
                            RCLCPP_ERROR(getLogger(), "Execution failed for group: %s", group_name.c_str());
                            all_success = false;
                        } else {
                            RCLCPP_INFO(getLogger(), "Execution succeeded for group: %s", group_name.c_str());
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(getLogger(), "Exception in execution thread for %s: %s", 
                                   group_name.c_str(), e.what());
                        std::lock_guard<std::mutex> lock(results_mutex);
                        execution_results[group_name] = false;
                        all_success = false;
                    }
                });
            }
            
            // 等待所有执行线程完成
            for (auto& thread : execution_threads) {
                thread.join();
            }
            
            // 检查所有组是否都有执行结果
            bool all_groups_executed = true;
            for (const auto& [group_name, _] : trajectories_by_group) {
                if (execution_results.find(group_name) == execution_results.end()) {
                    RCLCPP_ERROR(getLogger(), "Missing execution result for group: %s", group_name.c_str());
                    all_groups_executed = false;
                    all_success = false;
                }
            }
            
            if (all_success && all_groups_executed) {
                response["message"] = "Trajectory planning and execution completed";
            } else {
                response["success"] = false;
                response["error"] = "Some trajectories failed to execute";
                for (const auto& [group_name, success] : execution_results) {
                    if (!success) {
                        RCLCPP_ERROR(getLogger(), "Trajectory execution failed for group: %s", group_name.c_str());
                    }
                }
                if (!all_groups_executed) {
                    RCLCPP_ERROR(getLogger(), "Some groups did not report execution results");
                }
            }
        }
        
        sendResponse(response.dump());
        
        if (response["success"].get<bool>()) {
        successful_requests_++;
        } else {
            failed_requests_++;
        }
        
    } catch (const std::exception& e) {
        failed_requests_++;
        std::string error_msg = "Failed to process request: " + std::string(e.what());
        RCLCPP_ERROR(getLogger(), "%s", error_msg.c_str());
        sendErrorResponse(error_msg);
    }
}

void TrajectoryPlannerServer::sendResponse(const std::string& response_data) {
    try {
        zmq::message_t reply(response_data.size());
        memcpy(reply.data(), response_data.data(), response_data.size());
        zmq_socket_->send(reply, zmq::send_flags::none);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Failed to send response: %s", e.what());
    }
}

void TrajectoryPlannerServer::sendErrorResponse(const std::string& error_message) {
    nlohmann::json error_response;
    error_response["success"] = false;
    error_response["error"] = error_message;
    sendResponse(error_response.dump());
}

void TrajectoryPlannerServer::logStatistics() {
    RCLCPP_INFO(getLogger(), "Server Statistics:");
    RCLCPP_INFO(getLogger(), "  Total requests: %lu", total_requests_.load());
    RCLCPP_INFO(getLogger(), "  Successful: %lu", successful_requests_.load());
    RCLCPP_INFO(getLogger(), "  Failed: %lu", failed_requests_.load());
    
    if (total_requests_.load() > 0) {
        double success_rate = 100.0 * successful_requests_.load() / total_requests_.load();
        RCLCPP_INFO(getLogger(), "  Success rate: %.2f%%", success_rate);
    }
}

std::string TrajectoryPlannerServer::generateExecutionId() {
    uint64_t id = execution_id_counter_.fetch_add(1);
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(16) << id;
    return ss.str();
}

std::string TrajectoryPlannerServer::storePendingExecution(
    const std::map<std::string, moveit_msgs::msg::RobotTrajectory>& trajectories) {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    // 生成唯一的执行 ID（16 位十六进制）
    std::string execution_id = generateExecutionId();
    
    // 存储待执行的轨迹
    PendingExecution pending;
    pending.trajectories = trajectories;
    pending.timestamp = std::chrono::high_resolution_clock::now();
    
    pending_executions_[execution_id] = pending;
    
    // 清理过期的执行记录（超过 1 小时）
    auto now = std::chrono::high_resolution_clock::now();
    for (auto it = pending_executions_.begin(); it != pending_executions_.end();) {
        auto age = std::chrono::duration_cast<std::chrono::hours>(now - it->second.timestamp);
        if (age.count() > 1) {
            it = pending_executions_.erase(it);
        } else {
            ++it;
        }
    }
    
    return execution_id;
}

bool TrajectoryPlannerServer::executePendingTrajectory(const std::string& execution_id) {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    
    // 查找待执行的轨迹
    auto it = pending_executions_.find(execution_id);
    if (it == pending_executions_.end()) {
        RCLCPP_ERROR(getLogger(), "Execution ID not found: %s", execution_id.c_str());
        return false;
    }
    
    const auto& pending = it->second;
    bool all_success = true;
    std::vector<std::thread> execution_threads;
    std::map<std::string, bool> execution_results;
    std::mutex results_mutex;
    
    // 为每个组创建独立线程并行执行
    for (const auto& [group_name, trajectory] : pending.trajectories) {
        execution_threads.emplace_back([this, &results_mutex, &execution_results, &all_success, group_name, trajectory]() {
            try {
                // 为每个线程创建独立的节点，避免 ROS 2 executor 冲突
                auto executor_node = rclcpp::Node::make_shared(
                    "trajectory_executor_" + group_name + "_" + 
                    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())
                );
                
                TrajectoryExecutor executor(executor_node);
                bool success = executor.executeTrajectory(trajectory, group_name, true);
                
                // 记录执行结果（线程安全）
                std::lock_guard<std::mutex> lock(results_mutex);
                execution_results[group_name] = success;
                if (!success) {
                    all_success = false;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(getLogger(), "Exception in execution thread for %s: %s", 
                           group_name.c_str(), e.what());
                std::lock_guard<std::mutex> lock(results_mutex);
                execution_results[group_name] = false;
                all_success = false;
            }
        });
    }
    
    // 等待所有执行线程完成
    for (auto& thread : execution_threads) {
        thread.join();
    }
    
    // 检查所有组是否都有执行结果
    bool all_groups_executed = true;
    for (const auto& [group_name, _] : pending.trajectories) {
        if (execution_results.find(group_name) == execution_results.end()) {
            RCLCPP_ERROR(getLogger(), "Missing execution result for group: %s", group_name.c_str());
            all_groups_executed = false;
            all_success = false;
        }
    }
    
    // 删除已执行的记录
    pending_executions_.erase(it);
    
    if (!all_success || !all_groups_executed) {
        RCLCPP_ERROR(getLogger(), "Some trajectories failed to execute for execution_id: %s", execution_id.c_str());
        for (const auto& [group_name, success] : execution_results) {
            if (!success) {
                RCLCPP_ERROR(getLogger(), "  - %s: failed", group_name.c_str());
            }
        }
        if (!all_groups_executed) {
            RCLCPP_ERROR(getLogger(), "Some groups did not report execution results");
        }
    }
    
    return all_success && all_groups_executed;
}

moveit_msgs::msg::RobotTrajectory TrajectoryPlannerServer::jsonToRobotTrajectory(const nlohmann::json& traj_json) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    if (traj_json.contains("joint_names")) {
        trajectory.joint_trajectory.joint_names = traj_json["joint_names"].get<std::vector<std::string>>();
    }
    
    if (traj_json.contains("points")) {
        for (const auto& point_json : traj_json["points"]) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            
            if (point_json.contains("positions")) {
                point.positions = point_json["positions"].get<std::vector<double>>();
            }
            if (point_json.contains("velocities")) {
                point.velocities = point_json["velocities"].get<std::vector<double>>();
            }
            if (point_json.contains("accelerations")) {
                point.accelerations = point_json["accelerations"].get<std::vector<double>>();
            }
            if (point_json.contains("time_from_start")) {
                const auto& time_json = point_json["time_from_start"];
                point.time_from_start.sec = time_json.value("sec", 0);
                point.time_from_start.nanosec = time_json.value("nanosec", 0U);
            }
            
            trajectory.joint_trajectory.points.push_back(point);
        }
    }
    
    return trajectory;
}

} // namespace ur_move

