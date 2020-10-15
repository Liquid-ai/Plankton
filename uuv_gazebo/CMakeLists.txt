cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY rviz launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
