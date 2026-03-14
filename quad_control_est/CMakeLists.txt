cmake_minimum_required(VERSION 3.8)
project(quad_control_est)

# 消除 hpp-fcl/coal 警告 (必须放在 find_package(pinocchio) 之前)
add_definitions(-DCOAL_DISABLE_HPP_FCL_WARNINGS)

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

######

add_library(${PROJECT_NAME} SHARED
  src/StateEstimateBase.cpp
  src/LinearKalmanFilter.cpp
  src/FromTopiceEstimate.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Boost::filesystem
  Boost::system
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ocs2_core
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_legged_robot
  ocs2_robotic_tools
)

######

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  pinocchio 
  Boost 
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ocs2_core
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_legged_robot
  ocs2_robotic_tools
)

ament_package()
