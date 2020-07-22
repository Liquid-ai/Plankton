cmake_minimum_required(VERSION 3.5)
project(uuv_sensor_ros_plugins_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} 
  "msg/DVL.msg" 
  "msg/DVLBeam.msg"
  "msg/PositionWithCovariance.msg" 
  "msg/PositionWithCovarianceStamped.msg"
  "msg/ChemicalParticleConcentration.msg" 
  "msg/Salinity.msg"
  "srv/ChangeSensorState.srv"
  DEPENDENCIES 
  #builtin_interfaces 
  geometry_msgs
  std_msgs
)

ament_package()
