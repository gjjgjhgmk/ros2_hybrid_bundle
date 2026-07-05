#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#endif
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include "ur_move/waypoint_message.hpp"

namespace ur_move {

class MoveItPlanner {
public:
    explicit MoveItPlanner(rclcpp::Node::SharedPtr node);
    ~MoveItPlanner() = default;
    
    bool initialize();
    
    moveit_msgs::msg::RobotTrajectory planTrajectory(
        const std::vector<Waypoint>& waypoints);
    
    // 按组规划轨迹，返回每个组的轨迹
    std::map<std::string, moveit_msgs::msg::RobotTrajectory> planTrajectoriesByGroup(
        const std::vector<Waypoint>& waypoints);
    
    std::vector<std::string> getAvailablePlanners() const;

private:
    rclcpp::Node::SharedPtr node_;
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    
    std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> move_groups_;
    
    bool initialized_ = false;
    
    bool initializeRobotModel();
    bool initializePlanningSceneMonitor();
    bool initializeMoveGroups();
    
    moveit::planning_interface::MoveGroupInterface::Plan planSingleWaypoint(
        const Waypoint& waypoint);
    
    moveit_msgs::msg::RobotTrajectory concatenateTrajectories(
        const std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& plans);
    
    rclcpp::Logger getLogger() const { return node_->get_logger(); }
};

} // namespace ur_move
