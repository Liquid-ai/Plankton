cmake_minimum_required(VERSION 3.5)
project(uuv_teleop)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

catkin_install_python(PROGRAMS scripts/vehicle_teleop.py
  scripts/finned_uuv_teleop.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
