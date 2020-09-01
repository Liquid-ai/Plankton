cmake_minimum_required(VERSION 3.5)
project(uuv_control_utils)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS 
  scripts/set_timed_current_perturbation.py
  scripts/apply_link_wrench.py 
  scripts/disturbance_manager.py
  scripts/start_helical_trajectory.py 
  scripts/start_circular_trajectory.py
  scripts/set_thruster_state.py 
  scripts/trajectory_marker_publisher.py
  scripts/set_thruster_output_efficiency.py 
  scripts/send_waypoint_file.py
  scripts/set_gm_current_perturbation.py 
  scripts/set_scalar_parameter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
