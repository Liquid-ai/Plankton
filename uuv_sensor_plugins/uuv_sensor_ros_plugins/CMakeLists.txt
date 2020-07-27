# cmake_minimum_required(VERSION 3.5)
# project(uuv_sensor_ros_plugins)

# message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})
# # Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find_package(image_transport REQUIRED)
# # find_package(sensor_msgs REQUIRED)
# # find_package(geometry_msgs REQUIRED)
# # find_package(uuv_gazebo_plugins REQUIRED)
# # find_package(gazebo_plugins REQUIRED)
# # find_package(laser_geometry REQUIRED)
# # find_package(message_filters REQUIRED)
# # find_package(gazebo_dev REQUIRED)
# # find_package(uuv_sensor_ros_plugins_msgs REQUIRED)
# # find_package(rclcpp REQUIRED)
# # find_package(tf2 REQUIRED)
# # find_package(tf2_ros REQUIRED)
# # find_package(std_msgs REQUIRED)
# find_package(ament_cmake REQUIRED)

# # set(LIBS 
# #   image_transport
# #   sensor_msgs
# #   geometry_msgs
# #   uuv_gazebo_plugins
# #   gazebo_plugins
# #   laser_geometry
# #   message_filters
# #   gazebo_dev
# #   uuv_sensor_ros_plugins_msgs
# #   rclcpp
# #   tf2
# #   tf2_ros
# #   std_msgs
# #   ament_cmake
# # )

# find_package(Eigen3 REQUIRED)
# #find_package(OGRE REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(Protobuf REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)


# set(PROTOBUF_IMPORT_DIRS "")
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
#   endif()
# endforeach()

# message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

# set(GAZEBO_MSG_INCLUDE_DIRS)
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
#   endif()
# endforeach()

# message(STATUS "GAZEBO_MSG_INCLUDE_DIRS = " ${GAZEBO_MSG_INCLUDE_DIRS})

# #
# include_directories(
#   include
#   ${GAZEBO_MSG_INCLUDE_DIRS}
#   #${PROJECT_SOURCE_DIR}/include
#   ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
# )
# #

# #file(GLOB msgs msgs/*.proto)
# set(msgs
#   msgs/SensorDvl.proto
#   msgs/SensorImu.proto
#   msgs/SensorMagnetic.proto
#   msgs/SensorPressure.proto
#   msgs/SensorRpt.proto
#   ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
#   ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
# )

# set(UUV_SENSOR_ROS_PLUGINS_LIST "")

# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
# add_library(uuv_sensor_gazebo_msgs SHARED ${PROTO_SRCS})
# ament_target_dependencies(uuv_sensor_gazebo_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_sensor_gazebo_msgs)

# add_library(uuv_gazebo_ros_base_sensor_plugin src/ROSBasePlugin.cpp
#   src/ROSBaseSensorPlugin.cpp)
# ament_target_dependencies(uuv_gazebo_ros_base_sensor_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_base_sensor_plugin)

# add_library(uuv_gazebo_ros_gps_plugin src/GPSROSPlugin.cpp src/ROSBasePlugin.cpp
#   src/ROSBaseSensorPlugin.cpp)
#   ament_target_dependencies(uuv_gazebo_ros_gps_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_gps_plugin)

# add_library(uuv_gazebo_ros_pose_gt_plugin src/PoseGTROSPlugin.cpp
#   src/ROSBasePlugin.cpp src/ROSBaseModelPlugin.cpp)
#   ament_target_dependencies(uuv_gazebo_ros_pose_gt_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_pose_gt_plugin)

# add_library(uuv_gazebo_ros_subsea_pressure_plugin src/SubseaPressureROSPlugin.cpp
#   src/ROSBasePlugin.cpp src/ROSBaseModelPlugin.cpp)
#   ament_target_dependencies(uuv_gazebo_ros_subsea_pressure_plugin ${LIBS}
#   ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_subsea_pressure_plugin)

# add_library(uuv_gazebo_ros_dvl_plugin src/DVLROSPlugin.cpp src/ROSBasePlugin.cpp
#   src/ROSBaseModelPlugin.cpp)
#   ament_target_dependencies(uuv_gazebo_ros_dvl_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_dvl_plugin)

# add_library(uuv_gazebo_ros_magnetometer_plugin src/MagnetometerROSPlugin.cpp
#   src/ROSBasePlugin.cpp src/ROSBaseModelPlugin.cpp)
# ament_target_dependencies(uuv_gazebo_ros_magnetometer_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_magnetometer_plugin)

# add_library(uuv_gazebo_ros_cpc_plugin src/CPCROSPlugin.cpp src/ROSBasePlugin.cpp
#   src/ROSBaseModelPlugin.cpp)
# ament_target_dependencies(uuv_gazebo_ros_cpc_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_cpc_plugin)

# add_library(uuv_gazebo_ros_imu_plugin src/IMUROSPlugin.cpp src/ROSBasePlugin.cpp
#   src/ROSBaseModelPlugin.cpp)
# ament_target_dependencies(uuv_gazebo_ros_imu_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_imu_plugin)

# add_library(uuv_gazebo_ros_rpt_plugin src/RPTROSPlugin.cpp src/ROSBasePlugin.cpp
#   src/ROSBaseModelPlugin.cpp)
# ament_target_dependencies(uuv_gazebo_ros_rpt_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_rpt_plugin)

# add_library(uuv_gazebo_ros_camera_plugin src/UnderwaterCameraROSPlugin.cpp)
# ament_target_dependencies(uuv_gazebo_ros_camera_plugin ${LIBS} ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_camera_plugin)

# add_library(image_sonar_ros_plugin src/gazebo_ros_image_sonar.cpp)
# ament_target_dependencies(image_sonar_ros_plugin ${OGRE_LIBRARIES} ${LIBS}
#   ${GAZEBO_LIBRARIES})
# list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST image_sonar_ros_plugin)

# install(TARGETS ${UUV_SENSOR_ROS_PLUGINS_LIST}
#   DESTINATION lib/${PROJECT_NAME}
#   LIBRARY
#   DESTINATION lib)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING
#   PATTERN "*.h"
#   PATTERN "*~"
#   EXCLUDE)

# install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING
#   PATTERN "*.pb.*"
#   PATTERN "*~"
#   EXCLUDE)

# install(DIRECTORY meshes urdf
#   DESTINATION share/${PROJECT_NAME}
#   PATTERN "*~"
#   EXCLUDE)

# #TODO add tests
# # if(BUILD_TESTING)
# #   find_package(rostest REQUIRED)
# #   find_package(xacro REQUIRED)
# #   find_package(rosunit REQUIRED)
# #   foreach(T test/test_urdf_files.test)
# #     add_rostest(${T})
# #   endforeach()
# # endif()

# ament_export_dependencies(image_transport)
# ament_export_dependencies(sensor_msgs)
# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(uuv_gazebo_plugins)
# ament_export_dependencies(gazebo_plugins)
# ament_export_dependencies(laser_geometry)
# ament_export_dependencies(message_filters)
# ament_export_dependencies(gazebo_dev)
# ament_export_dependencies(uuv_sensor_ros_plugins_msgs)
# ament_export_dependencies(rclcpp)
# ament_export_dependencies(tf2)
# ament_export_dependencies(tf2_ros)
# ament_export_dependencies(std_msgs)
# ament_export_dependencies(ament_cmake)
# ament_export_include_directories(include)
# ament_export_libraries(uuv_sensor_gazebo_msgs uuv_gazebo_ros_base_model_plugin
#   uuv_gazebo_ros_base_sensor_plugin uuv_gazebo_ros_gps_plugin
#   uuv_gazebo_ros_pose_gt_plugin uuv_gazebo_ros_subsea_pressure_plugin
#   uuv_gazebo_ros_dvl_plugin uuv_gazebo_ros_magnetometer_plugin
#   uuv_gazebo_ros_cpc_plugin uuv_gazebo_ros_imu_plugin uuv_gazebo_ros_rpt_plugin
#   uuv_gazebo_ros_camera_plugin image_sonar_ros_plugin)

#ament_package()

#####







cmake_minimum_required(VERSION 3.5)
project(uuv_sensor_ros_plugins)

message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})
# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(uuv_sensor_ros_plugins_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS include ${image_transport_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${uuv_gazebo_plugins_INCLUDE_DIRS} ${gazebo_plugins_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS} ${message_filters_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS} ${uuv_sensor_ros_plugins_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${image_transport_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS}
  ${geometry_msgs_LIBRARY_DIRS} ${uuv_gazebo_plugins_LIBRARY_DIRS}
  ${gazebo_plugins_LIBRARY_DIRS} ${laser_geometry_LIBRARY_DIRS}
  ${message_filters_LIBRARY_DIRS} ${gazebo_dev_LIBRARY_DIRS}
  ${uuv_sensor_ros_plugins_msgs_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${image_transport_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${uuv_gazebo_plugins_LIBRARIES}
  ${gazebo_plugins_LIBRARIES} ${laser_geometry_LIBRARIES}
  ${message_filters_LIBRARIES} ${gazebo_dev_LIBRARIES}
  ${uuv_sensor_ros_plugins_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES} ${std_msgs_LIBRARIES} ${ament_cmake_LIBRARIES})

#add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(OGRE REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)

set(UUV_SENSOR_ROS_PLUGINS_LIST "")

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

#
include_directories(
  ${INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
)
#

#file(GLOB msgs msgs/*.proto)
set(msgs
  msgs/SensorDvl.proto
  msgs/SensorImu.proto
  msgs/SensorMagnetic.proto
  msgs/SensorPressure.proto
  msgs/SensorRpt.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(uuv_sensor_gazebo_msgs SHARED ${PROTO_SRCS})
target_link_libraries(uuv_sensor_gazebo_msgs ${PROTOBUF_LIBRARY})
  #${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_sensor_gazebo_msgs)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.pb.*"
  PATTERN "*~"
  EXCLUDE)

#message(FATAL_ERROR "PROTO MSGS = " ${PROTO_HDRS})

add_library(uuv_gazebo_ros_base_model_plugin src/ROSBasePlugin.cpp
  src/ROSBaseModelPlugin.cpp)
target_link_libraries(uuv_gazebo_ros_base_model_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_base_model_plugin)

add_library(uuv_gazebo_ros_base_sensor_plugin src/ROSBasePlugin.cpp
  src/ROSBaseSensorPlugin.cpp)
target_link_libraries(uuv_gazebo_ros_base_sensor_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_base_sensor_plugin)

add_library(uuv_gazebo_ros_gps_plugin src/GPSROSPlugin.cpp src/ROSBasePlugin.cpp
  src/ROSBaseSensorPlugin.cpp)
target_link_libraries(uuv_gazebo_ros_gps_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_gps_plugin)

add_library(uuv_gazebo_ros_pose_gt_plugin src/PoseGTROSPlugin.cpp
  src/ROSBasePlugin.cpp src/ROSBaseModelPlugin.cpp)
target_link_libraries(uuv_gazebo_ros_pose_gt_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_pose_gt_plugin)

add_library(uuv_gazebo_ros_subsea_pressure_plugin src/SubseaPressureROSPlugin.cpp
  src/ROSBasePlugin.cpp src/ROSBaseModelPlugin.cpp)
add_dependencies(uuv_gazebo_ros_subsea_pressure_plugin uuv_sensor_gazebo_msgs)
target_link_libraries(uuv_gazebo_ros_subsea_pressure_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_subsea_pressure_plugin)

add_library(uuv_gazebo_ros_dvl_plugin src/DVLROSPlugin.cpp src/ROSBasePlugin.cpp
  src/ROSBaseModelPlugin.cpp)
add_dependencies(uuv_gazebo_ros_dvl_plugin uuv_sensor_gazebo_msgs)
target_link_libraries(uuv_gazebo_ros_dvl_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_dvl_plugin)

add_library(uuv_gazebo_ros_magnetometer_plugin src/MagnetometerROSPlugin.cpp
  src/ROSBasePlugin.cpp src/ROSBaseModelPlugin.cpp)
target_link_libraries(uuv_gazebo_ros_magnetometer_plugin ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_magnetometer_plugin)

add_library(uuv_gazebo_ros_cpc_plugin src/CPCROSPlugin.cpp src/ROSBasePlugin.cpp
  src/ROSBaseModelPlugin.cpp)
target_link_libraries(uuv_gazebo_ros_cpc_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_cpc_plugin)

add_library(uuv_gazebo_ros_imu_plugin src/IMUROSPlugin.cpp src/ROSBasePlugin.cpp
  src/ROSBaseModelPlugin.cpp)
add_dependencies(uuv_gazebo_ros_imu_plugin uuv_sensor_gazebo_msgs)
target_link_libraries(uuv_gazebo_ros_imu_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_imu_plugin)

add_library(uuv_gazebo_ros_rpt_plugin src/RPTROSPlugin.cpp src/ROSBasePlugin.cpp
  src/ROSBaseModelPlugin.cpp)
add_dependencies(uuv_gazebo_ros_rpt_plugin uuv_sensor_gazebo_msgs)
target_link_libraries(uuv_gazebo_ros_rpt_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_rpt_plugin)

add_library(uuv_gazebo_ros_camera_plugin src/UnderwaterCameraROSPlugin.cpp)
target_link_libraries(uuv_gazebo_ros_camera_plugin ${LIBS} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_camera_plugin)

add_library(image_sonar_ros_plugin src/gazebo_ros_image_sonar.cpp)
target_link_libraries(image_sonar_ros_plugin ${OGRE_LIBRARIES} ${LIBS}
  ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST image_sonar_ros_plugin)

install(TARGETS ${UUV_SENSOR_ROS_PLUGINS_LIST}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY
  DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*~"
  EXCLUDE)



install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

#TODO add tests
# if(BUILD_TESTING)
#   find_package(rostest REQUIRED)
#   find_package(xacro REQUIRED)
#   find_package(rosunit REQUIRED)
#   foreach(T test/test_urdf_files.test)
#     add_rostest(${T})
#   endforeach()
# endif()

ament_export_dependencies(image_transport)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(uuv_gazebo_plugins)
ament_export_dependencies(gazebo_plugins)
ament_export_dependencies(laser_geometry)
ament_export_dependencies(message_filters)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(uuv_sensor_ros_plugins_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})
# ament_export_libraries(uuv_sensor_gazebo_msgs uuv_gazebo_ros_base_model_plugin
#   uuv_gazebo_ros_base_sensor_plugin uuv_gazebo_ros_gps_plugin
#   uuv_gazebo_ros_pose_gt_plugin uuv_gazebo_ros_subsea_pressure_plugin
#   uuv_gazebo_ros_dvl_plugin uuv_gazebo_ros_magnetometer_plugin
#   uuv_gazebo_ros_cpc_plugin uuv_gazebo_ros_imu_plugin uuv_gazebo_ros_rpt_plugin
#   uuv_gazebo_ros_camera_plugin image_sonar_ros_plugin ${LIBS})

ament_package()
