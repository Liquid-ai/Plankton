cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo_worlds)

find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch media worlds models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
