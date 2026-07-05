#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace ur_move {

class TrajectoryExecutor {
public:
    explicit TrajectoryExecutor(rclcpp::Node::SharedPtr node);
    ~TrajectoryExecutor();
    
    bool executeTrajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        const std::string& group_name,
        bool wait_for_completion = true);
    
    bool isActionServerAvailable(const std::string& group_name);

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    std::atomic<bool> executor_running_;
    
    std::map<std::string, rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr> action_clients_;
    
    // 备用机制：当 future 状态更新延迟时，通过回调检测 goal 是否被接受
    std::atomic<bool> goal_accepted_;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>> accepted_goal_handle_;
    std::mutex goal_handle_mutex_;
    
    void startExecutor();
    void stopExecutor();
    
    std::string getActionName(const std::string& group_name);
    std::vector<std::string> getJointNames(const std::string& group_name);
    
    // 计算基于轨迹点数量的动态超时时间
    std::chrono::seconds calculateTimeout(size_t num_points) const;
    trajectory_msgs::msg::JointTrajectory sanitizeTrajectoryTiming(
        const trajectory_msgs::msg::JointTrajectory& trajectory) const;
    
    void goalResponseCallback(
        const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle);
    
    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle,
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback);
    
    void resultCallback(
        const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result);
};

} // namespace ur_move
