#include "ur_move/moveit_planner.hpp"
#include <cmath>

namespace ur_move {

MoveItPlanner::MoveItPlanner(rclcpp::Node::SharedPtr node)
    : node_(node) {
}

bool MoveItPlanner::initialize() {
    if (initialized_) {
        return true;
    }
    
    RCLCPP_INFO(getLogger(), "Initializing MoveIt Planner...");
    
    if (!initializeRobotModel()) {
        RCLCPP_ERROR(getLogger(), "Failed to initialize robot model");
        return false;
    }
    
    if (!initializePlanningSceneMonitor()) {
        RCLCPP_ERROR(getLogger(), "Failed to initialize planning scene monitor");
        return false;
    }
    
    if (!initializeMoveGroups()) {
        RCLCPP_ERROR(getLogger(), "Failed to initialize move groups");
        return false;
    }
    
    initialized_ = true;
    RCLCPP_INFO(getLogger(), "MoveIt Planner initialized successfully");
    return true;
}

bool MoveItPlanner::initializeRobotModel() {
    try {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
        robot_model_ = robot_model_loader_->getModel();
        
        if (!robot_model_) {
            RCLCPP_ERROR(getLogger(), "Failed to load robot model");
            return false;
        }
        
        RCLCPP_INFO(getLogger(), "Robot model loaded: %s", robot_model_->getName().c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Exception while loading robot model: %s", e.what());
        return false;
    }
}

bool MoveItPlanner::initializePlanningSceneMonitor() {
    try {
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            node_, robot_model_loader_, "planning_scene_monitor");
        
        if (!planning_scene_monitor_) {
            RCLCPP_ERROR(getLogger(), "Failed to create planning scene monitor");
            return false;
        }
        
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();
        
        RCLCPP_INFO(getLogger(), "Planning scene monitor initialized");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Exception while initializing planning scene monitor: %s", e.what());
        return false;
    }
}

bool MoveItPlanner::initializeMoveGroups() {
    try {
        for (const auto& group_name : {"left_arm", "right_arm"}) {
            auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
            move_group->setPlanningTime(10.0);
            move_group->setNumPlanningAttempts(3);
            move_groups_[group_name] = move_group;
            RCLCPP_INFO(getLogger(), "Initialized move group: %s", group_name);
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Exception while initializing move groups: %s", e.what());
        return false;
    }
}

std::vector<std::string> MoveItPlanner::getAvailablePlanners() const {
    return {"ptp", "lin", "ompl"};
}

moveit_msgs::msg::RobotTrajectory MoveItPlanner::planTrajectory(
    const std::vector<Waypoint>& waypoints) {
    
    moveit_msgs::msg::RobotTrajectory result_trajectory;
    if (waypoints.empty()) {
        RCLCPP_ERROR(getLogger(), "Empty waypoints list");
        return result_trajectory;
    }
    
    auto trajectories_by_group = planTrajectoriesByGroup(waypoints);
    if (!trajectories_by_group.empty()) {
        result_trajectory = trajectories_by_group.begin()->second;
    }
    return result_trajectory;
}

std::map<std::string, moveit_msgs::msg::RobotTrajectory> MoveItPlanner::planTrajectoriesByGroup(
    const std::vector<Waypoint>& waypoints) {
    
    std::map<std::string, moveit_msgs::msg::RobotTrajectory> result;
    
    if (waypoints.empty()) {
        RCLCPP_ERROR(getLogger(), "Empty waypoints list");
        return result;
    }
    
    // 按group分组路径点
    std::map<std::string, std::vector<Waypoint>> waypoints_by_group;
    for (const auto& waypoint : waypoints) {
        waypoints_by_group[waypoint.getGroup()].push_back(waypoint);
    }
    
    // 为每个group分别规划轨迹
    for (const auto& [group_name, group_waypoints] : waypoints_by_group) {
        RCLCPP_INFO(getLogger(), "Planning %zu waypoints for group: %s", 
                   group_waypoints.size(), group_name.c_str());

        const auto move_group_it = move_groups_.find(group_name);
        if (move_group_it == move_groups_.end()) {
            RCLCPP_ERROR(getLogger(), "Unknown group: %s", group_name.c_str());
            return {};
        }
        const auto& move_group = move_group_it->second;
        move_group->setStartStateToCurrentState();
        
        // 依次规划每个路径点
        std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
        for (const auto& waypoint : group_waypoints) {
            RCLCPP_INFO(getLogger(), "Planning waypoint: %s", waypoint.getName().c_str());
            auto plan = planSingleWaypoint(waypoint);
            if (plan.trajectory_.joint_trajectory.points.empty()) {
                move_group->setStartStateToCurrentState();
                RCLCPP_ERROR(getLogger(), "Group %s planning failed: waypoint %s failed", 
                           group_name.c_str(), waypoint.getName().c_str());
                return {};
            }

            // Chain each segment from the previous segment endpoint. Planning every
            // waypoint from the live state and then concatenating produces jumps.
            auto next_start = move_group->getCurrentState(1.0);
            const auto& joint_trajectory = plan.trajectory_.joint_trajectory;
            if (!next_start || joint_trajectory.joint_names.size() !=
                                   joint_trajectory.points.back().positions.size()) {
                move_group->setStartStateToCurrentState();
                RCLCPP_ERROR(getLogger(), "Cannot construct chained start state for %s", group_name.c_str());
                return {};
            }
            next_start->setVariablePositions(
                joint_trajectory.joint_names, joint_trajectory.points.back().positions);
            next_start->update();
            move_group->setStartState(*next_start);
            plans.push_back(plan);
        }

        move_group->setStartStateToCurrentState();
        
        if (plans.empty()) {
            RCLCPP_ERROR(getLogger(), "Group %s: no valid plans generated", group_name.c_str());
            return {};
        }
    
        // 合并同一个人group的所有轨迹为一个连续轨迹
        result[group_name] = concatenateTrajectories(plans);
        RCLCPP_INFO(getLogger(), "Group %s: trajectory with %zu points", 
                   group_name.c_str(), result[group_name].joint_trajectory.points.size());
    }
    
    return result;
}

moveit::planning_interface::MoveGroupInterface::Plan MoveItPlanner::planSingleWaypoint(
    const Waypoint& waypoint) {
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto it = move_groups_.find(waypoint.getGroup());
    if (it == move_groups_.end()) {
        RCLCPP_ERROR(getLogger(), "Unknown group: %s", waypoint.getGroup().c_str());
        return plan;
    }
    
    auto move_group = it->second;
    std::string planner = waypoint.getPlanner();
    
    // 设置规划器
    if (planner == "ptp" || planner == "lin") {
        move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group->setPlannerId(planner == "ptp" ? "PTP" : "LIN");
    } else {
        move_group->setPlanningPipelineId("ompl");
        move_group->setPlannerId(planner == "ompl" ? "AnytimePathShortening" : planner);
    }
    
    move_group->setMaxVelocityScalingFactor(waypoint.getMaxVelocityScalingFactor());
    move_group->setMaxAccelerationScalingFactor(waypoint.getMaxAccelerationScalingFactor());
    
    // 根据路径点类型设置目标：joint / cart
    if (waypoint.getType() == WaypointType::Joint) {
        // 关节空间目标：直接设置关节角度
        std::map<std::string, double> joint_values;
        const auto& joint_names = waypoint.getJointNames();
        const auto& joint_values_vec = waypoint.getJointValues();
        
        if (joint_names.size() != joint_values_vec.size()) {
            RCLCPP_ERROR(getLogger(), "Joint names and values size mismatch");
            return plan;
        }
        
        for (size_t i = 0; i < joint_names.size(); ++i) {
            joint_values[joint_names[i]] = joint_values_vec[i];
        }
        move_group->setJointValueTarget(joint_values);
    } else {
        // 笛卡尔空间目标：设置末端执行器位姿
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = waypoint.getFrameId();
        const auto& pos = waypoint.getPosition();
        const auto& ori = waypoint.getOrientation();
        target_pose.pose.position.x = pos[0];
        target_pose.pose.position.y = pos[1];
        target_pose.pose.position.z = pos[2];
        target_pose.pose.orientation.x = ori[0];
        target_pose.pose.orientation.y = ori[1];
        target_pose.pose.orientation.z = ori[2];
        target_pose.pose.orientation.w = ori[3];
        move_group->setPoseTarget(target_pose, waypoint.getIkFrame());
    }
    
    // 执行规划
    if (move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        return moveit::planning_interface::MoveGroupInterface::Plan();
    }
    
    return plan;
}

moveit_msgs::msg::RobotTrajectory MoveItPlanner::concatenateTrajectories(
    const std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& plans) {
    
    if (plans.empty()) {
        return moveit_msgs::msg::RobotTrajectory();
    }
    
    // 使用第一个轨迹作为基础
    moveit_msgs::msg::RobotTrajectory result = plans[0].trajectory_;
    
    // 合并后续轨迹，调整时间戳使其连续
    for (size_t i = 1; i < plans.size(); ++i) {
        const auto& traj = plans[i].trajectory_;
        if (result.joint_trajectory.joint_names != traj.joint_trajectory.joint_names) {
            RCLCPP_WARN(getLogger(), "Joint names mismatch, skipping trajectory %zu", i);
            continue;
        }
        
        // 获取当前轨迹的最后一个点的时间
        double last_time = 0.0;
        if (!result.joint_trajectory.points.empty()) {
            const auto& last_point = result.joint_trajectory.points.back();
            last_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
        }
        
        // 将新轨迹的点添加到结果中，时间戳累加
        for (const auto& point : traj.joint_trajectory.points) {
            auto new_point = point;
            double point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            double new_time = last_time + point_time;
            new_point.time_from_start.sec = static_cast<int32_t>(new_time);
            new_point.time_from_start.nanosec = static_cast<uint32_t>((new_time - new_point.time_from_start.sec) * 1e9);
            result.joint_trajectory.points.push_back(new_point);
        }
    }
    
    return result;
}

} // namespace ur_move
