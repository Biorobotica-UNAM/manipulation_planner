cmake_minimum_required(VERSION 3.8)
project(manipulation_planner)

# -----------------------------
# Compiler options
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_planner_interfaces REQUIRED)

# -----------------------------
# Build node
# -----------------------------
add_executable(right_arm_ik_solver_node
  src/right_arm_ik_solver_node.cpp
)

ament_target_dependencies(right_arm_ik_solver_node
  rclcpp
  geometry_msgs
  manipulation_planner_interfaces
)

add_executable(trajectory_planner_node
  src/trajectory_planner_node.cpp
)

ament_target_dependencies(trajectory_planner_node
  rclcpp
  geometry_msgs
  manipulation_planner_interfaces
)

# -----------------------------
# Install
# -----------------------------
install(TARGETS right_arm_ik_solver_node trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

