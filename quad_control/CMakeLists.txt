cmake_minimum_required(VERSION 3.5)
project(quad_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  rviz
  config
  model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
