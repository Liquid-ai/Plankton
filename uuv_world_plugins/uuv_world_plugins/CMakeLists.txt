cmake_minimum_required(VERSION 3.5)
project(uuv_world_plugins)
find_package(gazebo_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

set(INCLUDE_DIRS include ${gazebo_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${gazebo_msgs_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${gazebo_dev_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${gazebo_msgs_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${gazebo_dev_LIBRARIES})

# Specify C++11 standard
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost REQUIRED)

###########
## Build ##
###########

add_library(uuv_underwater_current_plugin src/UnderwaterCurrentPlugin.cc
  src/GaussMarkovProcess.cc)
target_link_libraries(uuv_underwater_current_plugin ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES} ${LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS uuv_underwater_current_plugin
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.hh")

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uuv_world_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_dev)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(uuv_underwater_current_plugin ${LIBS})

ament_package()
