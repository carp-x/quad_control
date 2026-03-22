cmake_minimum_required(VERSION 3.5)
project(quad_control_wbc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(rclcpp REQUIRED)
find_package(quad_control_mpc REQUIRED)

find_path(QPOASES_INCLUDE_DIRS NAMES qpOASES.hpp PATH_SUFFIXES qpoases)
find_library(QPOASES_LIBRARIES NAMES qpOASES)
if(NOT QPOASES_INCLUDE_DIRS OR NOT QPOASES_LIBRARIES)
  message(FATAL_ERROR "qpOASES not found! Please check your source installation.")
endif()

######

add_library(${PROJECT_NAME} SHARED
  src/WbcBase.cpp
  src/WeightedWbc.cpp
  src/HierarchicalQp.cpp
  src/HierarchicalWbc.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${pinocchio_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${QPOASES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Eigen3::Eigen 
  Boost::filesystem
  Boost::system
  ${QPOASES_LIBRARIES} 
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
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
  rclcpp
  quad_control_mpc
  pinocchio
  Eigen3
  Boost
)

ament_export_libraries(${QPOASES_LIBRARIES})
ament_export_include_directories(include ${QPOASES_INCLUDE_DIRS})

######

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/HQp_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
