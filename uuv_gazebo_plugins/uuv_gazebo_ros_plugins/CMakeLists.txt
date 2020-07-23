# cmake_minimum_required(VERSION 3.5)
# project(uuv_gazebo_ros_plugins)

# # Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(tf2_ros REQUIRED)
# find_package(uuv_gazebo_plugins REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(gazebo_dev REQUIRED)

# set(BASE_LIBS 
#   tf2_ros 
#   uuv_gazebo_plugins 
#   geometry_msgs 
#   tf2 
#   std_msgs 
#   ament_cmake
#   sensor_msgs 
#   visualization_msgs 
#   uuv_gazebo_ros_plugins_msgs 
#   rclcpp 
#   gazebo_dev 
# )

# set(UUV_GAZEBO_ROS_PLUGINS_LIST "")

# add_library(uuv_fin_ros_plugin src/FinROSPlugin.cpp)
# ament_target_dependencies(
#   uuv_fin_ros_plugin
#   ${BASE_LIBS}
# )
# list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_fin_ros_plugin)

# add_library(uuv_thruster_ros_plugin src/ThrusterROSPlugin.cpp)
# ament_target_dependencies(uuv_thruster_ros_plugin ${BASE_LIBS})
# list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_thruster_ros_plugin)

# add_library(uuv_underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cpp)
# ament_target_dependencies(uuv_underwater_object_ros_plugin ${BASE_LIBS})
# list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_underwater_object_ros_plugin)

# add_library(uuv_joint_state_publisher src/JointStatePublisher.cpp)
# ament_target_dependencies(uuv_joint_state_publisher ${BASE_LIBS})
# list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_joint_state_publisher)

# add_library(uuv_accelerations_test_plugin src/AccelerationsTestPlugin.cpp)
# ament_target_dependencies(uuv_accelerations_test_plugin ${BASE_LIBS})
# list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_accelerations_test_plugin)

# add_library(linear_battery_ros_plugin src/LinearBatteryROSPlugin.cpp)
# ament_target_dependencies(linear_battery_ros_plugin ${BASE_LIBS})
# target_link_libraries(linear_battery_ros_plugin LinearBatteryPlugin)
# list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST linear_battery_ros_plugin)

# add_library(custom_battery_consumer_ros_plugin src/CustomBatteryConsumerROSPlugin.cpp)
# ament_target_dependencies(custom_battery_consumer_ros_plugin ${BASE_LIBS})
# list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST custom_battery_consumer_ros_plugin)


# set(EXPORT_MY_LIBS "")
# foreach(VAL ${UUV_GAZEBO_ROS_PLUGINS_LIST})
#   list(APPEND EXPORT_MY_LIBS "export_${VAL}")
# endforeach()

# #MESSAGE(FATAL_ERROR ${EXPORT_MY_LIBS})

# ament_export_interfaces(${EXPORT_MY_LIBS} HAS_LIBRARY_TARGET)#${EXPORT_MY_LIBS} HAS_LIBRARY_TARGET)
# ament_export_dependencies(${BASE_LIBS})
# ament_export_include_directories(${INCLUDE_DIRS})
# ament_export_libraries(${UUV_GAZEBO_ROS_PLUGINS_LIST})

# foreach(VAL ${UUV_GAZEBO_ROS_PLUGINS_LIST})
#   install(TARGETS ${VAL}
#     EXPORT export_${VAL}#${EXPORT_MY_LIBS}
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     RUNTIME DESTINATION bin/${PROJECT_NAME}
#     INCLUDES DESTINATION include
#   )
# endforeach()

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING
#   PATTERN "*.h"
#   PATTERN "*~"
#   EXCLUDE)

# install(DIRECTORY urdf
#   DESTINATION share/${PROJECT_NAME}
#   PATTERN "*~"
#   EXCLUDE)

# #TODO re-enable xacro...
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   #find_package(rostest REQUIRED)
#   #find_package(xacro REQUIRED)
#   #find_package(rosunit REQUIRED)
#   ament_add_gtest(test1 test/test_default_fossen_vehicle.test)
#   ament_add_gtest(test2 test/test_sphere_vehicle.test)
#   ament_add_gtest(test3 test/test_thrusters.test)
#   # foreach(T 
#   #   test/test_default_fossen_vehicle.test 
#   #   test/test_sphere_vehicle.test
#   #   test/test_thrusters.test)
#       #ament_add_gtest(${T})
#     #add_rostest(${T})
#   #endforeach()
# endif()

# ament_package()





cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo_ros_plugins)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(tf2_ros REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)

set(INCLUDE_DIRS include ${tf2_ros_INCLUDE_DIRS}
  ${uuv_gazebo_plugins_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${visualization_msgs_INCLUDE_DIRS}
  ${uuv_gazebo_ros_plugins_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${tf2_ros_LIBRARY_DIRS} ${uuv_gazebo_plugins_LIBRARY_DIRS}
  ${geometry_msgs_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS}
  ${visualization_msgs_LIBRARY_DIRS} ${uuv_gazebo_ros_plugins_msgs_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS} ${gazebo_dev_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${tf2_ros_LIBRARIES} ${uuv_gazebo_plugins_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${tf2_LIBRARIES} ${std_msgs_LIBRARIES}
  ${ament_cmake_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES} ${uuv_gazebo_ros_plugins_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES} ${gazebo_dev_LIBRARIES})

set(UUV_GAZEBO_ROS_PLUGINS_LIST "")

add_library(uuv_fin_ros_plugin src/FinROSPlugin.cpp)
# ament_target_dependencies(
#   uuv_fin_ros_plugin
#   tf2_ros 
#   uuv_gazebo_plugins 
#   geometry_msgs 
# tf2 
# std_msgs 
# ament_cmake
# sensor_msgs 
# visualization_msgs 
# uuv_gazebo_ros_plugins_msgs 
# rclcpp 
# gazebo_dev 
# )

target_link_libraries(uuv_fin_ros_plugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_fin_ros_plugin)

add_library(uuv_thruster_ros_plugin src/ThrusterROSPlugin.cpp)
target_link_libraries(uuv_thruster_ros_plugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_thruster_ros_plugin)

add_library(uuv_underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cpp)
target_link_libraries(uuv_underwater_object_ros_plugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_underwater_object_ros_plugin)

add_library(uuv_joint_state_publisher src/JointStatePublisher.cpp)
target_link_libraries(uuv_joint_state_publisher ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_joint_state_publisher)

add_library(uuv_accelerations_test_plugin src/AccelerationsTestPlugin.cpp)
target_link_libraries(uuv_accelerations_test_plugin ${GAZEBO_LIBRARIES} ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_accelerations_test_plugin)

add_library(linear_battery_ros_plugin src/LinearBatteryROSPlugin.cpp)
target_link_libraries(linear_battery_ros_plugin LinearBatteryPlugin ${LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST linear_battery_ros_plugin)

add_library(custom_battery_consumer_ros_plugin
  src/CustomBatteryConsumerROSPlugin.cpp)
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
  PATTERN "*.h"
  PATTERN "*~"
  EXCLUDE)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

#TODO re-enable xacro...
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(rostest REQUIRED)
  #find_package(xacro REQUIRED)
  #find_package(rosunit REQUIRED)
  ament_add_gtest(test1 test/test_default_fossen_vehicle.test)
  ament_add_gtest(test2 test/test_sphere_vehicle.test)
  ament_add_gtest(test3 test/test_thrusters.test)
  # foreach(T 
  #   test/test_default_fossen_vehicle.test 
  #   test/test_sphere_vehicle.test
  #   test/test_thrusters.test)
      #ament_add_gtest(${T})
    #add_rostest(${T})
  #endforeach()
endif()

ament_export_dependencies(tf2_ros)
ament_export_dependencies(uuv_gazebo_plugins)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(uuv_gazebo_ros_plugins_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(uuv_fin_ros_plugin uuv_thruster_ros_plugin
  uuv_underwater_object_ros_plugin uuv_joint_state_publisher
  uuv_accelerations_test_plugin linear_battery_ros_plugin
  custom_battery_consumer_ros_plugin ${LIBS})

ament_package()
