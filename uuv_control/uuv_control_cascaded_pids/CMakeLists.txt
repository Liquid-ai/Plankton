cmake_minimum_required(VERSION 3.5)
project(uuv_control_cascaded_pids)

find_package(ament_cmake REQUIRED)

ament_python_install_package(uuv_PID PACKAGE_DIR src/PID)

install(PROGRAMS scripts/acceleration_control.py
                 scripts/position_control.py 
                 scripts/velocity_control.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
