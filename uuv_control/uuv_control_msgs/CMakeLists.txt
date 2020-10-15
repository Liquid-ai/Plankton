cmake_minimum_required(VERSION 3.5)
project(uuv_control_msgs)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg" 
  "msg/Waypoint.msg" 
  "msg/WaypointSet.msg"
  "srv/AddWaypoint.srv" 
  "srv/ClearWaypoints.srv" 
  "srv/InitCircularTrajectory.srv"
  "srv/InitHelicalTrajectory.srv" 
  "srv/InitWaypointsFromFile.srv"
  "srv/GetWaypoints.srv" 
  "srv/GoTo.srv" 
  "srv/GoToIncremental.srv" 
  "srv/Hold.srv"
  "srv/IsRunningTrajectory.srv" 
  "srv/InitWaypointSet.srv"
  "srv/InitRectTrajectory.srv" 
  "srv/StartTrajectory.srv"
  "srv/SwitchToAutomatic.srv" 
  "srv/SwitchToManual.srv" 
  "srv/SetPIDParams.srv"
  "srv/GetPIDParams.srv" 
  "srv/SetSMControllerParams.srv"
  "srv/GetSMControllerParams.srv" 
  "srv/SetMBSMControllerParams.srv"
  "srv/GetMBSMControllerParams.srv" 
  "srv/ResetController.srv"
  DEPENDENCIES 
  builtin_interfaces 
  std_msgs 
  geometry_msgs)

ament_package()
