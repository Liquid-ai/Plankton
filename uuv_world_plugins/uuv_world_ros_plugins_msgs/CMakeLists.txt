cmake_minimum_required(VERSION 2.8.3)
project(uuv_world_ros_plugins_msgs)

find_package(ament_cmake REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetCurrentModel.srv
  GetCurrentModel.srv
  SetCurrentVelocity.srv
  SetCurrentDirection.srv
  SetOriginSphericalCoord.srv
  GetOriginSphericalCoord.srv
  TransformToSphericalCoord.srv
  TransformFromSphericalCoord.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_package(
#  INCLUDE_DIRS
#  LIBRARIES
   AMENT_DEPENDS std_msgs geometry_msgs message_runtime
#  DEPENDS
)
