cmake_minimum_required(VERSION 3.22)
project(manipulation_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# For generating custom interfaces
find_package(rosidl_default_generators REQUIRED)

# ============================================
# Generate custom service interfaces
# ============================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmMovementPlan.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Get the typesupport target for C++
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# ============================================
# Executable 1: Arm Movement Planner Service
# ============================================
add_executable(arm_movement_planner_node
  src/plan_arm_movement_service.cpp
)

ament_target_dependencies(arm_movement_planner_node
  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_geometry_msgs
)

# Link the generated service interface
target_link_libraries(arm_movement_planner_node "${cpp_typesupport_target}")

# CRITICAL: Ensure service is generated before building executable
# rosidl_get_typesupport_target(cpp_typesupport_target 
#   "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# ============================================
# Library: MoveIt to Hardware Bridge
# ============================================
add_library(moveit_to_hardware_bridge SHARED
  src/moveit_to_hardware_bridge.cpp
)

ament_target_dependencies(moveit_to_hardware_bridge
  rclcpp
  std_msgs
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# ============================================
# Install targets
# ============================================
install(TARGETS
  arm_movement_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS moveit_to_hardware_bridge
  DESTINATION lib
)

# Install plugin description
install(FILES moveit_to_hardware_bridge_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface moveit_to_hardware_bridge_plugin.xml)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/srv")
  install(
    DIRECTORY srv
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY srv DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
