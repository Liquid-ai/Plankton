cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo_ros_plugins)
find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)

set(INCLUDE_DIRS include ${uuv_gazebo_ros_plugins_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${visualization_msgs_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${uuv_gazebo_plugins_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${uuv_gazebo_ros_plugins_msgs_LIBRARY_DIRS}
  ${geometry_msgs_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS} ${visualization_msgs_LIBRARY_DIRS}
  ${gazebo_dev_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${uuv_gazebo_plugins_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${uuv_gazebo_ros_plugins_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES} ${rclcpp_LIBRARIES} ${tf2_LIBRARIES}
  ${visualization_msgs_LIBRARIES} ${gazebo_dev_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} ${ament_cmake_LIBRARIES} ${uuv_gazebo_plugins_LIBRARIES})

# Specify C++11 standard
add_definitions(-std=c++11)

set(UUV_GAZEBO_ROS_PLUGINS_LIST "")

add_library(uuv_fin_ros_plugin src/FinROSPlugin.cc)
target_link_libraries(uuv_fin_ros_plugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_fin_ros_plugin)

add_library(uuv_thruster_ros_plugin src/ThrusterROSPlugin.cc)
target_link_libraries(uuv_thruster_ros_plugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_thruster_ros_plugin)

add_library(uuv_underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cc)
target_link_libraries(uuv_underwater_object_ros_plugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_underwater_object_ros_plugin)

add_library(uuv_joint_state_publisher src/JointStatePublisher.cc)
target_link_libraries(uuv_joint_state_publisher ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_joint_state_publisher)

add_library(uuv_accelerations_test_plugin src/AccelerationsTestPlugin.cc)
target_link_libraries(uuv_accelerations_test_plugin ${GAZEBO_LIBRARIES} ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_accelerations_test_plugin)

add_library(linear_battery_ros_plugin src/LinearBatteryROSPlugin.cc)
target_link_libraries(linear_battery_ros_plugin LinearBatteryPlugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST linear_battery_ros_plugin)

add_library(custom_battery_consumer_ros_plugin
  src/CustomBatteryConsumerROSPlugin.cc)
target_link_libraries(custom_battery_consumer_ros_plugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST custom_battery_consumer_ros_plugin)

install(TARGETS ${UUV_GAZEBO_ROS_PLUGINS_LIST}
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.hh"
  PATTERN "*~"
  EXCLUDE)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

if(BUILD_TESTING)
  find_package(rostest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(rosunit REQUIRED)
  foreach(T test/test_default_fossen_vehicle.test test/test_sphere_vehicle.test
    test/test_thrusters.test)
    add_rostest(${T})
  endforeach()
endif()

ament_export_dependencies(uuv_gazebo_ros_plugins_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(uuv_gazebo_plugins)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(uuv_fin_ros_plugin uuv_thruster_ros_plugin
  uuv_underwater_object_ros_plugin uuv_joint_state_publisher
  uuv_accelerations_test_plugin linear_battery_ros_plugin
  custom_battery_consumer_ros_plugin ${LIBS})

ament_package()
