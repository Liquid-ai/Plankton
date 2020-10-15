cmake_minimum_required(VERSION 3.5)
project(uuv_thruster_manager)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} 
  "srv/GetThrusterCurve.srv"
  "srv/ThrusterManagerInfo.srv" 
  "srv/SetThrusterManagerConfig.srv"
  "srv/GetThrusterManagerConfig.srv"
  DEPENDENCIES 
  builtin_interfaces 
  std_msgs
)

# rosidl_target_interfaces(publish_address_book
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_python_install_package(uuv_thrusters PACKAGE_DIR src/uuv_thrusters)
ament_python_install_package(uuv_thrusters.models PACKAGE_DIR src/uuv_thrusters/models)


install(
  PROGRAMS scripts/thruster_allocator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)


if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  find_package(xacro REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Unit tests
  set(THRUSTER_PYTHON_UNIT_TESTS
      test/test_thrusters.py
  )

  foreach(T ${THRUSTER_PYTHON_UNIT_TESTS})
    get_filename_component(TEST_NAME ${T} NAME_WE)
    ament_add_pytest_test(${TEST_NAME} ${T}
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 120
    )   
  endforeach()

  # ROS tests
  set(THRUSTER_PYTHON_ROS_TESTS
    test/test_thruster_allocator_x_axis.test.py
    test/test_thruster_allocator_y_axis.test.py
    test/test_thruster_allocator_z_axis.test.py
    test/test_thruster_manager_proportional_correct.test.py
  )

  foreach(T ${THRUSTER_PYTHON_ROS_TESTS})
    get_filename_component(TEST_NAME ${T} NAME_WE)
    add_launch_test(
      ${T}
      TARGET ${TEST_NAME}
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 120
    )
  endforeach()
endif()

ament_package()
