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

rosidl_generate_interfaces(
  ${PROJECT_NAME} 
  "msg/AUVCommand.msg"
  DEPENDENCIES 
  #builtin_interfaces ?
  geometry_msgs 
  std_msgs
)

ament_python_install_package(uuv_auv_actuator_interface PACKAGE_DIR src/uuv_auv_actuator_interface)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE
)

install(
  PROGRAMS scripts/control_allocator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
