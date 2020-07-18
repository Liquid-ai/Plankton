cmake_minimum_required(VERSION 3.5)
project(uuv_auv_control_allocator)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/AUVCommand.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)

set(INCLUDE_DIRS ${geometry_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${geometry_msgs_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${rosidl_default_generators_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${geometry_msgs_LIBRARIES} ${std_msgs_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${ament_cmake_LIBRARIES})

catkin_install_python(PROGRAMS scripts/control_allocator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
