cmake_minimum_required(VERSION 3.8)
project(multi_bot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
