cmake_minimum_required(VERSION 3.5)
project(uuv_tutorial_seabed_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
