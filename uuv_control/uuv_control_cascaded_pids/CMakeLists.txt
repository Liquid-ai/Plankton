cmake_minimum_required(VERSION 2.8.3)
project(uuv_control_cascaded_pid)

find_package(ament_cmake REQUIRED COMPONENTS
  dynamic_reconfigure)

ament_python_setup()

generate_dynamic_reconfigure_options(
  cfg/AccelerationControl.cfg
  cfg/PositionControl.cfg
  cfg/VelocityControl.cfg
)

# make sure configure headers are built before any node using them
#add_dependencies(example_node ${PROJECT_NAME}_gencfg)

ament_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

ament_install_python(PROGRAMS scripts/AccelerationControl.py
                               scripts/PositionControl.py
                               scripts/VelocityControl.py
                      DESTINATION ${AMENT_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${AMENT_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
