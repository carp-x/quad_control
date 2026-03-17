cmake_minimum_required(VERSION 3.5)
project(quad_control_gz)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

#########

add_library(${PROJECT_NAME} SHARED
  src/QuadGzSimSystem.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  # gz-sim::core
  gz-sim::gz-sim
)

ament_target_dependencies(${PROJECT_NAME}
  std_msgs  
  geometry_msgs
  control_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  control_toolbox
  gz_ros2_control
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(gz_ros2_control plugins.xml)

#########

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
