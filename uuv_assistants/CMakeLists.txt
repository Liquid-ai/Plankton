cmake_minimum_required(VERSION 3.5)
project(uuv_assistants)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(uuv_message_to_tf src/message_to_tf.cpp)

ament_target_dependencies(
  uuv_message_to_tf 
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs 
  nav_msgs 
  ament_cmake 
  rclcpp
)

ament_python_install_package(tf_quaternion PACKAGE_DIR src/tf_quaternion)

install(DIRECTORY launch templates
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)

# Needs to be installed in lib and not bin folder in order to be found by ament
install(TARGETS uuv_message_to_tf
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS scripts/publish_footprints.py
          scripts/set_simulation_timer.py 
          scripts/publish_world_models.py
          scripts/publish_vehicle_footprint.py 
          scripts/unpause_simulation.py
          scripts/create_new_robot_model 
          scripts/create_thruster_manager_configuration
          scripts/create_control_package
  DESTINATION lib/${PROJECT_NAME})

ament_package()
