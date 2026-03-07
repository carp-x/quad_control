cmake_minimum_required(VERSION 3.5.0)
project(quad_control_gz)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(gz-sim REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(control_toolbox REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)

#########

add_library(${PROJECT_NAME} SHARED
  src/QuadGaSimSystem.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    # gz-sim::core
    gz-sim::gz-sim
)

ament_target_dependencies(${PROJECT_NAME}
  std_msgs  
  geometry_msgs
  control_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  control_toolbox
  gz_ros2_control
  hardware_interface
)

#########

## Install
install(DIRECTORY
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include/
)
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz-sim
  std_msgs  
  geometry_msgs
  control_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  control_toolbox
  gz_ros2_control
  hardware_interface
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} ${PROJECT_NAME}_plugins.xml)

ament_package()
