cmake_minimum_required(VERSION 3.5)
project(uuv_thruster_manager)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/GetThrusterCurve.srv"
  "srv/ThrusterManagerInfo.srv" "srv/SetThrusterManagerConfig.srv"
  "srv/GetThrusterManagerConfig.srv"
  DEPENDENCIES builtin_interfaces std_msgs)

set(INCLUDE_DIRS ${std_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${std_msgs_LIBRARY_DIRS}
  ${rosidl_default_generators_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${std_msgs_LIBRARIES} ${rosidl_default_generators_LIBRARIES}
  ${ament_cmake_LIBRARIES})

catkin_install_python(PROGRAMS scripts/thruster_allocator.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

if(BUILD_TESTING)
  find_package(rostest REQUIRED)
  foreach(T test/test_thrusters.test test/test_thruster_allocator_x_axis.test
    test/test_thruster_allocator_y_axis.test
    test/test_thruster_allocator_z_axis.test
    test/test_thruster_manager_proportional_correct.test)
    add_rostest(${T})
  endforeach()
endif()

ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
