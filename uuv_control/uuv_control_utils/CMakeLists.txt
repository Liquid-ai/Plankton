cmake_minimum_required(VERSION 3.5)
project(uuv_control_utils)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

# set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
# include_directories(${INCLUDE_DIRS})

# set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

# link_directories(${LIBRARY_DIRS})

# set(LIBS ${ament_cmake_LIBRARIES})

# catkin_install_python(PROGRAMS scripts/set_timed_current_perturbation.py
#   scripts/apply_body_wrench.py scripts/disturbance_manager.py
#   scripts/start_helical_trajectory.py scripts/start_circular_trajectory.py
#   scripts/set_thruster_state.py scripts/trajectory_marker_publisher.py
#   scripts/set_thruster_output_efficiency.py scripts/send_waypoint_file.py
#   scripts/set_gm_current_perturbation.py scripts/set_scalar_parameter.py
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

# ament_export_dependencies(ament_cmake)
# ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
