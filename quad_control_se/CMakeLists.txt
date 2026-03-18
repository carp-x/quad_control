cmake_minimum_required(VERSION 3.5)
project(quad_control_se)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(quad_control_mpc REQUIRED)

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
    ${pinocchio_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Eigen3::Eigen 
  Boost::filesystem
  Boost::system
)

ament_target_dependencies(${PROJECT_NAME}
  nav_msgs
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ocs2_core
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_tools
  quad_control_mpc
)

######

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  nav_msgs
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ocs2_core
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_tools
  quad_control_mpc
  pinocchio 
  Eigen3
  Boost
)

ament_package()
