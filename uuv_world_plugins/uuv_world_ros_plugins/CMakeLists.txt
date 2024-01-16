cmake_minimum_required(VERSION 3.5)
project(uuv_world_ros_plugins)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(uuv_world_plugins REQUIRED)
find_package(uuv_world_ros_plugins_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

set(LIBS
  uuv_world_plugins
  uuv_world_ros_plugins_msgs
  gazebo_msgs
  gazebo_dev
  ament_cmake
  geometry_msgs
  rclcpp
  gazebo_ros
)

include_directories(include)

add_library(uuv_underwater_current_ros_plugin SHARED src/UnderwaterCurrentROSPlugin.cpp)
ament_target_dependencies(uuv_underwater_current_ros_plugin ${LIBS})

add_library(uuv_sc_ros_interface_plugin SHARED src/SphericalCoordinatesROSInterfacePlugin.cpp)
ament_target_dependencies(uuv_sc_ros_interface_plugin ${LIBS})

install(
  TARGETS uuv_underwater_current_ros_plugin
  EXPORT  export_uuv_underwater_current_ros_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(
  TARGETS uuv_sc_ros_interface_plugin
  EXPORT  export_uuv_sc_ros_interface_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".~"
  EXCLUDE
)

ament_export_targets(export_uuv_underwater_current_ros_plugin) #To use the namespace way :: with target_link_lib
ament_export_targets(export_uuv_sc_ros_interface_plugin)
ament_export_dependencies(uuv_world_plugins)
ament_export_dependencies(uuv_world_ros_plugins_msgs)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_export_libraries(
  uuv_underwater_current_ros_plugin
  uuv_sc_ros_interface_plugin
)

ament_package()
