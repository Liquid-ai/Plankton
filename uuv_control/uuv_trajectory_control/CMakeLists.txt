cmake_minimum_required(VERSION 3.5)
project(uuv_trajectory_control)

# Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

find_package(ament_cmake REQUIRED)

# set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
# include_directories(${INCLUDE_DIRS})

# set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

# link_directories(${LIBRARY_DIRS})

# set(LIBS ${ament_cmake_LIBRARIES})


ament_python_install_package(uuv_control_interfaces PACKAGE_DIR src/uuv_control_interfaces)
ament_python_install_package(uuv_trajectory_generator PACKAGE_DIR src/uuv_trajectory_generator)
ament_python_install_package(uuv_waypoints PACKAGE_DIR src/uuv_waypoints)
ament_python_install_package(uuv_trajectory_generator.path_generator PACKAGE_DIR src/uuv_trajectory_generator/path_generator)

install(PROGRAMS 
scripts/rov_nl_pid_controller.py
  scripts/rov_mb_sm_controller.py 
  scripts/rov_nmb_sm_controller.py
  scripts/rov_pid_controller.py 
  scripts/rov_ua_pid_controller.py
  scripts/rov_mb_fl_controller.py 
  scripts/auv_geometric_tracking_controller.py
  scripts/demo_wp_trajectory_generator.py
  scripts/rov_pd_grav_compensation_controller.py 
  scripts/rov_sf_controller.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

#TODO Re-enable testing
# if(BUILD_TESTING)
#   find_package(rostest REQUIRED)
#   foreach(T test/test_python_api.test)
#     add_rostest(${T})
#   endforeach()
# endif()

# ament_export_dependencies(ament_cmake)
# ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
