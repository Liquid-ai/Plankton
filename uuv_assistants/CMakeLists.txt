cmake_minimum_required(VERSION 3.5)
project(uuv_assistants)

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

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(topic_tools REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${topic_tools_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS})

set(LIBRARY_DIRS ${rclcpp_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS} ${topic_tools_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS})

set(LIBS ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${topic_tools_LIBRARIES} ${ament_cmake_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES})

add_executable(uuv_message_to_tf src/message_to_tf.cc)

target_link_libraries(uuv_message_to_tf ${LIBS})

catkin_install_python(PROGRAMS scripts/publish_footprints.py
  scripts/set_simulation_timer.py scripts/publish_world_models.py
  scripts/publish_vehicle_footprint.py scripts/unpause_simulation.py
  scripts/create_new_robot_model scripts/create_thruster_manager_configuration
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} PATTERN "*~" EXCLUDE)

install(TARGETS uuv_message_to_tf
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(topic_tools)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(tf2)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
