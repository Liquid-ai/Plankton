cmake_minimum_required(VERSION 3.5)
project(uuv_world_plugins)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(uuv_underwater_current_plugin SHARED 
  src/UnderwaterCurrentPlugin.cpp
  src/GaussMarkovProcess.cpp
)

ament_target_dependencies(uuv_underwater_current_plugin
  gazebo_dev
  gazebo_msgs
  ament_cmake
)

install(
  TARGETS uuv_underwater_current_plugin
  EXPORT export_uuv_underwater_current_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_export_interfaces(export_uuv_underwater_current_plugin) #To use the namespace way :: with target_link_lib
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(uuv_underwater_current_plugin)

ament_package()


###



# cmake_minimum_required(VERSION 3.5)
# project(uuv_world_plugins)

# # Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(gazebo_dev REQUIRED)
# find_package(gazebo_msgs REQUIRED)
# find_package(ament_cmake REQUIRED)

# set(INCLUDE_DIRS include ${gazebo_dev_INCLUDE_DIRS} ${gazebo_msgs_INCLUDE_DIRS}
#   ${ament_cmake_INCLUDE_DIRS})
# include_directories(${INCLUDE_DIRS})

# set(LIBRARY_DIRS ${gazebo_dev_LIBRARY_DIRS} ${gazebo_msgs_LIBRARY_DIRS}
#   ${ament_cmake_LIBRARY_DIRS})

# link_directories(${LIBRARY_DIRS})

# set(LIBS ${gazebo_dev_LIBRARIES} ${gazebo_msgs_LIBRARIES}
#   ${ament_cmake_LIBRARIES})

# find_package(Boost REQUIRED)

# add_library(uuv_underwater_current_plugin src/UnderwaterCurrentPlugin.cpp
#   src/GaussMarkovProcess.cpp)
# target_link_libraries(uuv_underwater_current_plugin ${GAZEBO_LIBRARIES}
#   ${Boost_LIBRARIES} ${LIBS})

# install(TARGETS uuv_underwater_current_plugin
#   ARCHIVE
#   DESTINATION lib
#   LIBRARY
#   DESTINATION lib
#   RUNTIME
#   DESTINATION bin/${PROJECT_NAME})

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING
#   PATTERN "*.h")

# ament_export_dependencies(gazebo_dev)
# ament_export_dependencies(gazebo_msgs)
# ament_export_dependencies(ament_cmake)
# ament_export_include_directories(${INCLUDE_DIRS})
# ament_export_libraries(uuv_underwater_current_plugin ${LIBS})

# ament_package()
