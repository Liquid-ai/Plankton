cmake_minimum_required(VERSION 3.5)
project(uuv_tutorial_dp_controller)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/tutorial_dp_controller.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
