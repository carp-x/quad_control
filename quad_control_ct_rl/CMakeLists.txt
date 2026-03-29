cmake_minimum_required(VERSION 3.5)
project(quad_control_ct_rl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(angles REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(quad_control_mpc REQUIRED)
find_package(quad_control_ros REQUIRED)
find_package(quad_control_se REQUIRED)
find_package(quad_control_wbc REQUIRED)
find_package(quad_control_ct REQUIRED)
find_package(pluginlib REQUIRED)

#########

add_library(${PROJECT_NAME} SHARED
  src/QuadControllerRL.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  onnxruntime::onnxruntime
  pinocchio::pinocchio
  Eigen3::Eigen 
  Boost::filesystem
  Boost::system
)

ament_target_dependencies(${PROJECT_NAME}
  angles
  ocs2_msgs
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  controller_interface
  hardware_interface
  ocs2_core
  ocs2_mpc
  ocs2_sqp
  ocs2_ddp
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_ros_interfaces
  quad_control_mpc
  quad_control_ros
  quad_control_se
  quad_control_wbc
  quad_control_ct
  pluginlib
)

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

#########

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
