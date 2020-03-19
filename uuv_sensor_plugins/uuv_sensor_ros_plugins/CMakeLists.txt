cmake_minimum_required(VERSION 3.5)
project(uuv_sensor_ros_plugins)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(uuv_sensor_ros_plugins_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)

set(INCLUDE_DIRS include ${geometry_msgs_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS} ${gazebo_plugins_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}
  ${uuv_sensor_ros_plugins_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${uuv_gazebo_plugins_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${geometry_msgs_LIBRARY_DIRS} ${gazebo_dev_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS} ${message_filters_LIBRARY_DIRS}
  ${image_transport_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS}
  ${gazebo_plugins_LIBRARY_DIRS} ${laser_geometry_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS} ${uuv_sensor_ros_plugins_msgs_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS}
  ${uuv_gazebo_plugins_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${geometry_msgs_LIBRARIES} ${gazebo_dev_LIBRARIES} ${tf2_LIBRARIES}
  ${std_msgs_LIBRARIES} ${message_filters_LIBRARIES} ${image_transport_LIBRARIES}
  ${tf2_ros_LIBRARIES} ${gazebo_plugins_LIBRARIES} ${laser_geometry_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${uuv_sensor_ros_plugins_msgs_LIBRARIES}
  ${ament_cmake_LIBRARIES} ${rclcpp_LIBRARIES} ${uuv_gazebo_plugins_LIBRARIES})

add_definitions(-std=c++11)

# set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Eigen3 REQUIRED)
find_package(OGRE REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(UUV_SENSOR_ROS_PLUGINS_LIST "")

# Add Gazebo custom protobuf messages
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

message(STATUS "GAZEBO_MSG_INCLUDE_DIRS = " ${GAZEBO_MSG_INCLUDE_DIRS})

file(GLOB msgs msgs/*.proto)

###########
## Build ##
###########

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(uuv_sensor_gazebo_msgs SHARED ${PROTO_SRCS})
target_link_libraries(uuv_sensor_gazebo_msgs ${PROTOBUF_LIBRARY}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_sensor_gazebo_msgs)

###############################################################################

add_library(uuv_gazebo_ros_base_model_plugin src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_base_model_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_base_model_plugin)

###############################################################################

add_library(uuv_gazebo_ros_base_sensor_plugin src/ROSBasePlugin.cc
  src/ROSBaseSensorPlugin.cc)
target_link_libraries(uuv_gazebo_ros_base_sensor_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_base_sensor_plugin)

###############################################################################

add_library(uuv_gazebo_ros_gps_plugin src/GPSROSPlugin.cc src/ROSBasePlugin.cc
  src/ROSBaseSensorPlugin.cc)
target_link_libraries(uuv_gazebo_ros_gps_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_gps_plugin)

###############################################################################

add_library(uuv_gazebo_ros_pose_gt_plugin src/PoseGTROSPlugin.cc
  src/ROSBasePlugin.cc src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_pose_gt_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_pose_gt_plugin)

###############################################################################

add_library(uuv_gazebo_ros_subsea_pressure_plugin src/SubseaPressureROSPlugin.cc
  src/ROSBasePlugin.cc src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_subsea_pressure_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_subsea_pressure_plugin)

###############################################################################

add_library(uuv_gazebo_ros_dvl_plugin src/DVLROSPlugin.cc src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_dvl_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_dvl_plugin)

###############################################################################

add_library(uuv_gazebo_ros_magnetometer_plugin src/MagnetometerROSPlugin.cc
  src/ROSBasePlugin.cc src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_magnetometer_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_magnetometer_plugin)

###############################################################################

add_library(uuv_gazebo_ros_cpc_plugin src/CPCROSPlugin.cc src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_cpc_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_cpc_plugin)

###############################################################################

add_library(uuv_gazebo_ros_imu_plugin src/IMUROSPlugin.cc src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_imu_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_imu_plugin)

###############################################################################

add_library(uuv_gazebo_ros_rpt_plugin src/RPTROSPlugin.cc src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_rpt_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_rpt_plugin)

###############################################################################

add_library(uuv_gazebo_ros_camera_plugin src/UnderwaterCameraROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_camera_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_camera_plugin)

###############################################################################

add_library(image_sonar_ros_plugin src/gazebo_ros_image_sonar.cpp)
target_link_libraries(image_sonar_ros_plugin ${OGRE_LIBRARIES} ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST image_sonar_ros_plugin)

#############
## Install ##
#############

install(TARGETS ${UUV_SENSOR_ROS_PLUGINS_LIST}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY
  DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.hh"
  PATTERN "*~"
  EXCLUDE)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.pb.*"
  PATTERN "*~"
  EXCLUDE)

# Install mesh files
install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

if(BUILD_TESTING)
  find_package(rostest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(rosunit REQUIRED)
  foreach(T test/test_urdf_files.test)
    add_rostest(${T})
  endforeach()
endif()

ament_export_dependencies(geometry_msgs)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(tf2)
ament_export_dependencies(std_msgs)
ament_export_dependencies(message_filters)
ament_export_dependencies(image_transport)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(gazebo_plugins)
ament_export_dependencies(laser_geometry)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(uuv_sensor_ros_plugins_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(uuv_gazebo_plugins)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(uuv_sensor_gazebo_msgs uuv_gazebo_ros_base_model_plugin
  uuv_gazebo_ros_base_sensor_plugin uuv_gazebo_ros_gps_plugin
  uuv_gazebo_ros_pose_gt_plugin uuv_gazebo_ros_subsea_pressure_plugin
  uuv_gazebo_ros_dvl_plugin uuv_gazebo_ros_magnetometer_plugin
  uuv_gazebo_ros_cpc_plugin uuv_gazebo_ros_imu_plugin uuv_gazebo_ros_rpt_plugin
  uuv_gazebo_ros_camera_plugin image_sonar_ros_plugin ${LIBS})

ament_package()
