cmake_minimum_required(VERSION 3.8)
project(ur_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)

# Find ZMQ using pkg-config
pkg_check_modules(ZMQ REQUIRED libzmq)

# Include directories
include_directories(include)

# Create trajectory planner server executable
add_executable(trajectory_planner_server
  src/server_cpp/trajectory_planner_server_main.cpp
  src/server_cpp/trajectory_planner_server.cpp
  src/server_cpp/waypoint_message.cpp
  src/server_cpp/trajectory_executor.cpp
  src/server_cpp/moveit_planner.cpp
)

# Link libraries
ament_target_dependencies(trajectory_planner_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  trajectory_msgs
  control_msgs
  geometry_msgs
  nlohmann_json
)

target_link_libraries(trajectory_planner_server
  ${ZMQ_LIBRARIES}
)

target_include_directories(trajectory_planner_server PRIVATE
  ${ZMQ_INCLUDE_DIRS}
)

# Install targets
install(TARGETS
  trajectory_planner_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install client scripts (clients and utilities)
install(PROGRAMS 
  client/zmq_ur_move_client.py 
  client/zmq_gripper_client.py 
  client/zmq_tf_client.py
  client/trajectory_executor_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install server scripts (gripper_control.py 已合并到 gripper_zmq_server.py)
install(PROGRAMS 
  src/server_py/gripper_zmq_server.py 
  src/server_py/tf_zmq_server.py
  src/server_py/joint_states_zmq_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install test files
install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
  FILES_MATCHING PATTERN "*.json" PATTERN "*.py" PATTERN "*.md"
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

