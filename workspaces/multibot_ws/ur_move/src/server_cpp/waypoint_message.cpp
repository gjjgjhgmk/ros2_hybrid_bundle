#include "ur_move/waypoint_message.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace ur_move {

// 将通用关节名称（joint1-joint6 或 left_joint1/right_joint1）映射到实际的 UR 关节名称
static std::vector<std::string> mapGenericJointNames(
    const std::vector<std::string>& generic_names,
    const std::string& group) {
    
    // UR 机器人的标准关节顺序
    static const std::vector<std::string> ur_joint_base_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    
    // 根据组名确定默认前缀（用于 joint1-joint6 格式）
    std::string default_prefix = (group == "left_arm") ? "left_" : 
                                 (group == "right_arm") ? "right_" : "";
    
    std::vector<std::string> mapped_names;
    mapped_names.reserve(generic_names.size());
    
    for (const auto& name : generic_names) {
        std::string prefix = "";
        std::string joint_part = name;
        int joint_id = -1;
        
        // 检查是否是 left_joint1 或 right_joint1 格式
        if (name.length() >= 10) {
            if (name.substr(0, 5) == "left_") {
                prefix = "left_";
                joint_part = name.substr(5);
            } else if (name.substr(0, 6) == "right_") {
                prefix = "right_";
                joint_part = name.substr(6);
            }
        }
        
        // 解析关节编号（joint1-joint6）
        if (joint_part.length() == 6 && joint_part.substr(0, 5) == "joint") {
            try {
                joint_id = std::stoi(joint_part.substr(5));
            } catch (...) {}
        } else if (name.length() == 6 && name.substr(0, 5) == "joint") {
            prefix = default_prefix;
            try {
                joint_id = std::stoi(name.substr(5));
            } catch (...) {}
        }
        
        // 如果成功解析出关节ID，映射到实际关节名称；否则保持原样
        if (joint_id >= 1 && joint_id <= 6) {
            mapped_names.push_back(prefix + ur_joint_base_names[joint_id - 1]);
        } else {
            mapped_names.push_back(name);
        }
    }
    
    return mapped_names;
}

nlohmann::json Waypoint::toJson() const {
    nlohmann::json json;
    
    json["name"] = name_;
    json["type"] = (type_ == WaypointType::Joint) ? "joint" : "cart";
    json["group"] = group_;
    json["planner"] = planner_;
    json["max_velocity_scaling_factor"] = max_velocity_scaling_factor_;
    json["max_acceleration_scaling_factor"] = max_acceleration_scaling_factor_;
    json["joint_names"] = joint_names_;
    
    if (type_ == WaypointType::Joint && !joint_values_.empty()) {
        std::vector<double> joint_values_degrees;
        joint_values_degrees.reserve(joint_values_.size());
        for (double radian_value : joint_values_) {
            joint_values_degrees.push_back(radian_value * 180.0 / M_PI);
        }
        json["joint_values"] = joint_values_degrees;
    } else {
        json["joint_values"] = joint_values_;
    }
    
    json["frame_id"] = frame_id_;
    json["ik_frame"] = ik_frame_;
    json["position"] = position_;
    json["orientation"] = orientation_;
    return json;
}

Waypoint Waypoint::fromJson(const nlohmann::json& json) {
    Waypoint waypoint;
    waypoint.setName(json.at("name").get<std::string>());
    waypoint.setGroup(json.at("group").get<std::string>());
    waypoint.setPlanner(json.at("planner").get<std::string>());
    waypoint.setMaxVelocityScalingFactor(json.value("max_velocity_scaling_factor", 0.1));
    waypoint.setMaxAccelerationScalingFactor(json.value("max_acceleration_scaling_factor", 0.1));
    
    std::string type_str = json.at("type").get<std::string>();
    if (type_str == "joint") {
        // 关节空间路径点：关节名称和角度值
        waypoint.setType(WaypointType::Joint);
        std::vector<std::string> generic_joint_names = json.at("joint_names").get<std::vector<std::string>>();
        // 将通用关节名称映射到实际关节名称
        waypoint.setJointNames(mapGenericJointNames(generic_joint_names, waypoint.getGroup()));
        
        // 将输入的角度值转换为弧度值（MoveIt 使用弧度）
        std::vector<double> joint_values_degrees = json.at("joint_values").get<std::vector<double>>();
        std::vector<double> joint_values_radians;
        joint_values_radians.reserve(joint_values_degrees.size());
        for (double degree_value : joint_values_degrees) {
            joint_values_radians.push_back(degree_value * M_PI / 180.0);
        }
        waypoint.setJointValues(joint_values_radians);
        
        waypoint.setFrameId(json.value("frame_id", "empty"));
        waypoint.setIkFrame(json.value("ik_frame", "empty"));
        waypoint.setPosition(json.value("position", std::array<double, 3>{0.0, 0.0, 0.0}));
        waypoint.setOrientation(json.value("orientation", std::array<double, 4>{0.0, 0.0, 0.0, 1.0}));
    } else if (type_str == "cart") {
        // 笛卡尔空间路径点：位置和姿态
        waypoint.setType(WaypointType::Cartesian);
        waypoint.setFrameId(json.at("frame_id").get<std::string>());
        waypoint.setIkFrame(json.at("ik_frame").get<std::string>());
        waypoint.setPosition(json.at("position").get<std::array<double, 3>>());
        waypoint.setOrientation(json.at("orientation").get<std::array<double, 4>>());
        waypoint.setJointNames(json.value("joint_names", std::vector<std::string>{}));
        waypoint.setJointValues(json.value("joint_values", std::vector<double>{}));
    } else {
        throw std::invalid_argument("Invalid waypoint type: " + type_str);
    }
    
    return waypoint;
}

void WaypointMessage::addWaypoint(const Waypoint& waypoint) {
    waypoints_.push_back(waypoint);
}

nlohmann::json WaypointMessage::toJson() const {
    nlohmann::json json;
    json["waypoints"] = nlohmann::json::array();
    
    for (const auto& waypoint : waypoints_) {
        json["waypoints"].push_back(waypoint.toJson());
    }
    
    return json;
}

WaypointMessage WaypointMessage::fromJson(const nlohmann::json& json) {
    WaypointMessage message;
    
    if (json.contains("waypoints") && json["waypoints"].is_array()) {
        for (const auto& waypoint_json : json["waypoints"]) {
            message.addWaypoint(Waypoint::fromJson(waypoint_json));
        }
    }
    
    return message;
}

std::string WaypointMessage::serialize() const {
    return toJson().dump();
}

WaypointMessage WaypointMessage::deserialize(const std::string& data) {
    try {
        nlohmann::json json = nlohmann::json::parse(data);
        return fromJson(json);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to deserialize waypoint message: " + std::string(e.what()));
    }
}

} // namespace ur_move

