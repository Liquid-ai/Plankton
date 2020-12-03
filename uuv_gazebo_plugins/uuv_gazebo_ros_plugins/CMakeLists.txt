cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo_ros_plugins)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)
find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(BASE_LIBS  
  ament_cmake 
  gazebo_dev
  geometry_msgs 
  rclcpp 
  sensor_msgs 
  std_msgs 
  tf2 
  tf2_ros 
  uuv_gazebo_plugins 
  uuv_gazebo_ros_plugins_msgs 
  visualization_msgs 
)

include_directories(include)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(UUV_GAZEBO_ROS_PLUGINS_LIST "")

add_library(uuv_fin_ros_plugin SHARED src/FinROSPlugin.cpp)
ament_target_dependencies(
  uuv_fin_ros_plugin
  ${BASE_LIBS}
)
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_fin_ros_plugin)

add_library(uuv_thruster_ros_plugin SHARED src/ThrusterROSPlugin.cpp)
ament_target_dependencies(uuv_thruster_ros_plugin ${BASE_LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_thruster_ros_plugin)

add_library(uuv_underwater_object_ros_plugin SHARED src/UnderwaterObjectROSPlugin.cpp)
ament_target_dependencies(uuv_underwater_object_ros_plugin ${BASE_LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_underwater_object_ros_plugin)

add_library(uuv_joint_state_publisher SHARED src/JointStatePublisher.cpp)
ament_target_dependencies(uuv_joint_state_publisher ${BASE_LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_joint_state_publisher)

add_library(uuv_accelerations_test_plugin SHARED src/AccelerationsTestPlugin.cpp)
ament_target_dependencies(uuv_accelerations_test_plugin ${BASE_LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST uuv_accelerations_test_plugin)

add_library(linear_battery_ros_plugin SHARED src/LinearBatteryROSPlugin.cpp)
ament_target_dependencies(linear_battery_ros_plugin ${BASE_LIBS})
# Check if it is necessary
target_link_libraries(linear_battery_ros_plugin LinearBatteryPlugin)
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST linear_battery_ros_plugin)

add_library(custom_battery_consumer_ros_plugin SHARED src/CustomBatteryConsumerROSPlugin.cpp)
ament_target_dependencies(custom_battery_consumer_ros_plugin ${BASE_LIBS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST custom_battery_consumer_ros_plugin)


set(EXPORT_MY_LIBS "")
foreach(VAL ${UUV_GAZEBO_ROS_PLUGINS_LIST})
  list(APPEND EXPORT_MY_LIBS "export_${VAL}")
endforeach()

#MESSAGE(FATAL_ERROR ${EXPORT_MY_LIBS})

ament_export_targets(${EXPORT_MY_LIBS})#${EXPORT_MY_LIBS} HAS_LIBRARY_TARGET)
ament_export_dependencies(${BASE_LIBS})
ament_export_include_directories(include)
ament_export_libraries(${UUV_GAZEBO_ROS_PLUGINS_LIST})

foreach(VAL ${UUV_GAZEBO_ROS_PLUGINS_LIST})
  install(TARGETS ${VAL}
    EXPORT export_${VAL}#${EXPORT_MY_LIBS}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    INCLUDES DESTINATION include
  )
endforeach()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*~"
  EXCLUDE)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(xacro REQUIRED)

  set(GAZEBO_ROS_PLUGINS_PYTHON_TESTS
    test/test_default_fossen_vehicle.test.py
    test/test_sphere_vehicle.test.py
    test/test_thrusters.test.py
  )

  foreach(T ${GAZEBO_ROS_PLUGINS_PYTHON_TESTS})
    get_filename_component(_test_name ${T} NAME_WE)
    add_launch_test(
      ${T}
      TARGET ${TEST_NAME}
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 120
    )
  endforeach()
endif()

ament_package()
