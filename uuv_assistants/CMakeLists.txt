cmake_minimum_required(VERSION 3.5)
project(uuv_assistants)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plankton_utils REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(uuv_message_to_tf src/message_to_tf.cpp)

ament_target_dependencies(
  uuv_message_to_tf 
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs 
  nav_msgs 
  ament_cmake 
  rclcpp
)

ament_python_install_package(tf_quaternion PACKAGE_DIR src/tf_quaternion)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)

#Needs to be installed in lib and not bin folder in order to be found by ament
install(TARGETS uuv_message_to_tf
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS scripts/publish_footprints.py
          scripts/set_simulation_timer.py 
          scripts/publish_world_models.py
          scripts/publish_vehicle_footprint.py 
          scripts/unpause_simulation.py
          scripts/create_new_robot_model 
          scripts/create_thruster_manager_configuration
  DESTINATION lib/${PROJECT_NAME})

# catkin_install_python(PROGRAMS scripts/publish_footprints.py
#   scripts/set_simulation_timer.py scripts/publish_world_models.py
#   scripts/publish_vehicle_footprint.py scripts/unpause_simulation.py
#   scripts/create_new_robot_model scripts/create_thruster_manager_configuration
#   DESTINATION lib/${PROJECT_NAME})

ament_package()



# cmake_minimum_required(VERSION 3.5)
# project(uuv_assistants)

# # Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# find_package(geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)

# set(INCLUDE_DIRS ${geometry_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
#   ${sensor_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS})
# include_directories(${INCLUDE_DIRS})

# set(LIBRARY_DIRS ${geometry_msgs_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS} ${tf2_geometry_msgs_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
#   ${sensor_msgs_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
#   ${rclcpp_LIBRARY_DIRS})

# link_directories(${LIBRARY_DIRS})

# set(LIBS ${geometry_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES}
#   ${nav_msgs_LIBRARIES} ${ament_cmake_LIBRARIES}
#   ${rclcpp_LIBRARIES})

# add_executable(uuv_message_to_tf src/message_to_tf.cpp)

# target_link_libraries(uuv_message_to_tf ${LIBS})

# # catkin_install_python(PROGRAMS scripts/publish_footprints.py
# #   scripts/set_simulation_timer.py scripts/publish_world_models.py
# #   scripts/publish_vehicle_footprint.py scripts/unpause_simulation.py
# #   scripts/create_new_robot_model scripts/create_thruster_manager_configuration
# #   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
#   PATTERN "*~"
#   EXCLUDE)

# install(TARGETS uuv_message_to_tf
#   ARCHIVE
#   DESTINATION lib
#   LIBRARY
#   DESTINATION lib
#   RUNTIME
#   DESTINATION lib/${PROJECT_NAME})

# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(tf2)
# ament_export_dependencies(sensor_msgs)
# ament_export_dependencies(nav_msgs)
# ament_export_dependencies(ament_cmake)
# ament_export_dependencies(rclcpp)
# ament_export_include_directories(${INCLUDE_DIRS})

# ament_package()
