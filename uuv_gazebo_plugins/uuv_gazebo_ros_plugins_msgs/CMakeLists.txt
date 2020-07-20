cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo_ros_plugins_msgs)

# Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

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
  #builtin_interfaces 
  std_msgs 
  geometry_msgs
)

# set(INCLUDE_DIRS ${rosidl_default_generators_INCLUDE_DIRS}
#   ${std_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
#   ${ament_cmake_INCLUDE_DIRS})
# include_directories(${INCLUDE_DIRS})

# set(LIBRARY_DIRS ${rosidl_default_generators_LIBRARY_DIRS}
#   ${std_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS}
#   ${ament_cmake_LIBRARY_DIRS})

# link_directories(${LIBRARY_DIRS})

# set(LIBS ${rosidl_default_generators_LIBRARIES} ${std_msgs_LIBRARIES}
#   ${geometry_msgs_LIBRARIES} ${ament_cmake_LIBRARIES})

# ament_export_dependencies(rosidl_default_generators)
# ament_export_dependencies(std_msgs)
# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(ament_cmake)
# ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
