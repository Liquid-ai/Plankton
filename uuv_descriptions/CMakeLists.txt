cmake_minimum_required(VERSION 3.5)
project(uuv_descriptions)

find_package(ament_cmake REQUIRED)

# TODO Check it this wrapper is still useful
# install(PROGRAMS scripts/spawn_model
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch meshes robots urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)

  #ament_lint_auto_find_test_dependencies()

  set(DESCRIPTIONS_PYTHON_TESTS
      test/test_urdf_files.py
  )

  foreach(T ${DESCRIPTIONS_PYTHON_TESTS})
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
