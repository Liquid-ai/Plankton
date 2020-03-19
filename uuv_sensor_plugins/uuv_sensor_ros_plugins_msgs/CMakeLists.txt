cmake_minimum_required(VERSION 3.5)
project(uuv_sensor_ros_plugins_msgs)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/DVL.msg" "msg/DVLBeam.msg"
  "msg/PositionWithCovariance.msg" "msg/PositionWithCovarianceStamped.msg"
  "msg/ChemicalParticleConcentration.msg" "msg/Salinity.msg"
  "srv/ChangeSensorState.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS}
  ${rosidl_default_generators_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
