cmake_minimum_required(VERSION 3.5)
project(uuv_world_ros_plugins_msgs)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} 
  "srv/SetCurrentModel.srv"
  "srv/GetCurrentModel.srv" 
  "srv/SetCurrentVelocity.srv"
  "srv/SetCurrentDirection.srv" 
  "srv/SetOriginSphericalCoord.srv"
  "srv/GetOriginSphericalCoord.srv" 
  "srv/TransformToSphericalCoord.srv"
  "srv/TransformFromSphericalCoord.srv"
  DEPENDENCIES 
  # builtin_interfaces 
  # std_msgs 
  geometry_msgs
)

ament_package()
