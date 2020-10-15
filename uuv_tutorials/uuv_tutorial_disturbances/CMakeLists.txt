cmake_minimum_required(VERSION 3.5)
project(uuv_tutorial_disturbances)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
