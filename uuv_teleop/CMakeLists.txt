cmake_minimum_required(VERSION 3.5)
project(uuv_teleop)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)

install(
  PROGRAMS 
  scripts/vehicle_teleop.py
  scripts/vehicle_keyboard_teleop.py
  scripts/finned_uuv_teleop.py
  DESTINATION lib/${PROJECT_NAME}) #python executables are only found in lib folders by ros2 run (?!)...

ament_package()
