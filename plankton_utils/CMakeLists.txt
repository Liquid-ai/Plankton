cmake_minimum_required(VERSION 3.5)
project(plankton_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(plankton_global_sim_time src/globalSimTime.cpp)

ament_target_dependencies(
  plankton_global_sim_time
  ament_cmake 
  rclcpp
)

# Needs to be installed in lib and not bin folder in order to be found by ament
install(TARGETS plankton_global_sim_time 
  DESTINATION lib/${PROJECT_NAME}
)


ament_python_install_package(plankton_utils PACKAGE_DIR src/plankton_utils)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  set(plankton_utils_python_tests
      test/test_time.py
      test/test_param_helper.py
  )

  foreach(_test_path ${plankton_utils_python_tests})
    #ament_add_pytest_test(${PROJECT_NAME}_pytests ${_test_path}) #change for uniqueness
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
        APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
        PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 120
        WERROR ON
      )
    endforeach()
endif()

ament_package()
