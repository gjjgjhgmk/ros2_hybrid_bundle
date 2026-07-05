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
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(intent_hybrid_interfaces REQUIRED)
find_package(ompl REQUIRED)

add_executable(intent_runtime_bridge
  src/intent_runtime_bridge.cpp
  src/collision_checker.cpp
  src/intent_rrt_connect.cpp
  src/ompl_rrt_connect.cpp
)
target_include_directories(intent_runtime_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
)
ament_target_dependencies(intent_runtime_bridge
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  visualization_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  intent_hybrid_interfaces
)
target_link_libraries(intent_runtime_bridge
  ${OMPL_LIBRARIES}
)

target_compile_features(intent_runtime_bridge PUBLIC cxx_std_17)

install(TARGETS intent_runtime_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
