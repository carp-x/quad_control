cmake_minimum_required(VERSION 3.5)
project(quad_control_ct)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

#########

add_library(${PROJECT_NAME} SHARED
  src/QuadController.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  controller_interface
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

#########

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

  # ament_export_include_directories(
  #   include
  # )
  # ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  # ament_export_dependencies(
  #   rclcpp
  #   rclcpp_lifecycle
  #   realtime_tools
  #   controller_interface
  #   hardware_interface
  #   pluginlib
  # )

ament_package()
