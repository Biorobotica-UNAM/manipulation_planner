cmake_minimum_required(VERSION 3.8)
project(manipulation_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_planner_interfaces REQUIRED)

add_executable(manipulation_planner_node
  src/manipulation_planner_node.cpp
)

ament_target_dependencies(manipulation_planner_node
  rclcpp
  geometry_msgs
  manipulation_planner_interfaces
)

install(TARGETS manipulation_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

