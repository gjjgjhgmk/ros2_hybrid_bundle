#include <rclcpp/rclcpp.hpp>
#include "ur_move/trajectory_planner_server.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("trajectory_planner_server");
    int bind_port = 5605;
    node->declare_parameter<int>("bind_port", bind_port);
    bind_port = node->get_parameter("bind_port").as_int();
    
    ur_move::TrajectoryPlannerServer server(node);
    if (!server.start(bind_port)) {
        RCLCPP_ERROR(rclcpp::get_logger("trajectory_planner_server"), 
                    "Failed to start trajectory planner server");
        rclcpp::shutdown();
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("trajectory_planner_server"), 
               "Trajectory Planner Server running on port %d", bind_port);
    
    rclcpp::spin(node);
    server.stop();
    rclcpp::shutdown();
    
    return 0;
}

