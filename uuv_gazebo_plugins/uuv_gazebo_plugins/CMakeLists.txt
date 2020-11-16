cmake_minimum_required(VERSION 3.5)
project(uuv_gazebo_plugins)
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS} ${gazebo_dev_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${gazebo_dev_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${gazebo_dev_LIBRARIES})

#add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)

set(UUV_GAZEBO_PLUGINS_LIST "")

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

include_directories(
  ${INCLUDE_DIRS} 
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
)

file(GLOB msgs msgs/*.proto)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(uuv_gazebo_plugins_msgs SHARED ${PROTO_SRCS})
target_link_libraries(uuv_gazebo_plugins_msgs ${PROTOBUF_LIBRARY})
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_gazebo_plugins_msgs)

add_library(uuv_dynamics SHARED src/Dynamics.cpp)
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_dynamics)

add_library(uuv_fin_plugin SHARED src/LiftDragModel.cpp src/FinPlugin.cpp)
target_link_libraries(uuv_fin_plugin uuv_dynamics uuv_gazebo_plugins_msgs
  ${LIBS} ${Boost_LIBRARIES})
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_fin_plugin)

add_library(uuv_underwater_object_plugin SHARED src/HydrodynamicModel.cpp
  src/BuoyantObject.cpp src/UnderwaterObjectPlugin.cpp)
target_link_libraries(uuv_underwater_object_plugin ${LIBS} ${EIGEN3_LIBRARIES})
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_underwater_object_plugin)

add_library(uuv_thruster_plugin SHARED src/ThrusterConversionFcn.cpp
  src/ThrusterPlugin.cpp)
target_link_libraries(uuv_thruster_plugin uuv_dynamics uuv_gazebo_plugins_msgs
  ${LIBS} ${Boost_LIBRARIES})
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_thruster_plugin)

add_library(umbilical_plugin SHARED src/UmbilicalModel.cpp src/UmbilicalPlugin.cpp)
target_link_libraries(umbilical_plugin ${LIBS} ${Boost_LIBRARIES})
list(APPEND UUV_GAZEBO_PLUGINS_LIST umbilical_plugin)

install(TARGETS ${UUV_GAZEBO_PLUGINS_LIST}
  ARCHIVE
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

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN ".h")

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.pb.*"
  PATTERN "*~"
  EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(UNIT_ThrusterConversionFcn_TEST
    src/ThrusterConversionFcn_TEST.cpp)
  target_link_libraries(UNIT_ThrusterConversionFcn_TEST uuv_thruster_plugin
    ${LIBS})

  ament_add_gtest(UNIT_ThrusterDynamics_TEST src/ThrusterDynamics_TEST.cpp)
  target_link_libraries(UNIT_ThrusterDynamics_TEST uuv_dynamics ${LIBS})
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_dev)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(uuv_gazebo_plugins_msgs uuv_dynamics uuv_fin_plugin
  uuv_underwater_object_plugin uuv_thruster_plugin umbilical_plugin)

ament_package()
