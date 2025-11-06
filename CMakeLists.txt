cmake_minimum_required(VERSION 3.22)
project(manipulation_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

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

install(TARGETS moveit_to_hardware_bridge
  DESTINATION lib
)

# Install plugin description
install(FILES moveit_to_hardware_bridge_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface moveit_to_hardware_bridge_plugin.xml)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
