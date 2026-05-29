cmake_minimum_required(VERSION 3.8)
project(intent_hybrid_runtime_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(intent_hybrid_interfaces REQUIRED)

add_executable(intent_runtime_bridge src/intent_runtime_bridge.cpp)
ament_target_dependencies(intent_runtime_bridge
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  moveit_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  intent_hybrid_interfaces
)

target_compile_features(intent_runtime_bridge PUBLIC cxx_std_17)

install(TARGETS intent_runtime_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
