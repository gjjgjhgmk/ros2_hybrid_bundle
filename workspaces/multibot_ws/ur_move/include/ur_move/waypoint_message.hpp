#pragma once

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <nlohmann/json.hpp>

namespace ur_move {

enum class WaypointType {
    Joint,
    Cartesian
};

class Waypoint {
public:
    Waypoint() = default;
    
    // Getters
    const std::string& getName() const { return name_; }
    WaypointType getType() const { return type_; }
    const std::string& getGroup() const { return group_; }
    const std::string& getPlanner() const { return planner_; }
    double getMaxVelocityScalingFactor() const { return max_velocity_scaling_factor_; }
    double getMaxAccelerationScalingFactor() const { return max_acceleration_scaling_factor_; }
    
    // Joint waypoint specific
    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    const std::vector<double>& getJointValues() const { return joint_values_; }
    
    // Cartesian waypoint specific
    const std::string& getFrameId() const { return frame_id_; }
    const std::string& getIkFrame() const { return ik_frame_; }
    const std::array<double, 3>& getPosition() const { return position_; }
    const std::array<double, 4>& getOrientation() const { return orientation_; }
    
    // Setters
    void setName(const std::string& name) { name_ = name; }
    void setType(WaypointType type) { type_ = type; }
    void setGroup(const std::string& group) { group_ = group; }
    void setPlanner(const std::string& planner) { planner_ = planner; }
    void setMaxVelocityScalingFactor(double factor) { max_velocity_scaling_factor_ = factor; }
    void setMaxAccelerationScalingFactor(double factor) { max_acceleration_scaling_factor_ = factor; }
    
    void setJointNames(const std::vector<std::string>& names) { joint_names_ = names; }
    void setJointValues(const std::vector<double>& values) { joint_values_ = values; }
    
    void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
    void setIkFrame(const std::string& ik_frame) { ik_frame_ = ik_frame; }
    void setPosition(const std::array<double, 3>& position) { position_ = position; }
    void setOrientation(const std::array<double, 4>& orientation) { orientation_ = orientation; }
    
    // JSON serialization
    nlohmann::json toJson() const;
    static Waypoint fromJson(const nlohmann::json& json);

private:
    std::string name_;
    WaypointType type_ = WaypointType::Joint;
    std::string group_;
    std::string planner_;
    double max_velocity_scaling_factor_ = 0.1;
    double max_acceleration_scaling_factor_ = 0.1;
    
    // Joint waypoint data
    std::vector<std::string> joint_names_;
    std::vector<double> joint_values_;
    
    // Cartesian waypoint data
    std::string frame_id_;
    std::string ik_frame_;
    std::array<double, 3> position_ = {0.0, 0.0, 0.0};
    std::array<double, 4> orientation_ = {0.0, 0.0, 0.0, 1.0};
};

class WaypointMessage {
public:
    WaypointMessage() = default;
    
    void addWaypoint(const Waypoint& waypoint);
    const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
    void clearWaypoints() { waypoints_.clear(); }
    
    // JSON serialization
    nlohmann::json toJson() const;
    static WaypointMessage fromJson(const nlohmann::json& json);
    
    // String serialization for ZMQ
    std::string serialize() const;
    static WaypointMessage deserialize(const std::string& data);

private:
    std::vector<Waypoint> waypoints_;
};

} // namespace ur_move

