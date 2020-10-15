cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo_ros_plugins_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} 
  "msg/FloatStamped.msg"
  "msg/UnderwaterObjectModel.msg" 
  "msg/ThrusterConversionFcn.msg"
  "srv/SetThrusterEfficiency.srv" 
  "srv/SetThrusterState.srv"
  "srv/GetThrusterEfficiency.srv" 
  "srv/GetThrusterState.srv"
  "srv/SetUseGlobalCurrentVel.srv" 
  "srv/GetModelProperties.srv" 
  "srv/SetFloat.srv"
  "srv/GetFloat.srv" 
  "srv/GetListParam.srv" 
  "srv/GetThrusterConversionFcn.srv"
  DEPENDENCIES 
  # builtin_interfaces 
  std_msgs 
  geometry_msgs
)

ament_package()
