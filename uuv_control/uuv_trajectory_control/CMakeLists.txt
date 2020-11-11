cmake_minimum_required(VERSION 3.5)
project(uuv_trajectory_control)

find_package(ament_cmake REQUIRED)

ament_python_install_package(uuv_control_interfaces PACKAGE_DIR src/uuv_control_interfaces)
ament_python_install_package(uuv_trajectory_generator PACKAGE_DIR src/uuv_trajectory_generator)
ament_python_install_package(uuv_waypoints PACKAGE_DIR src/uuv_waypoints)
ament_python_install_package(uuv_trajectory_generator.path_generator PACKAGE_DIR src/uuv_trajectory_generator/path_generator)
ament_python_install_package(utils PACKAGE_DIR src/utils)

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
  PATTERN "*~" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  set(TRAJECTORY_PYTHON_TESTS
      test/test_waypoint.py
      test/test_trajectory_point.py
      test/test_waypoint_set.py
  )

  foreach(T ${TRAJECTORY_PYTHON_TESTS})
    get_filename_component(_test_name ${T} NAME_WE)
    ament_add_pytest_test(${_test_name} ${T}
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 120
    )   
  endforeach()
endif()

ament_package()
