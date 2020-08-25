cmake_minimum_required(VERSION 3.5)
project(uuv_control_cascaded_pid)

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

# generate_dynamic_reconfigure_options(cfg/AccelerationControl.cfg
#   cfg/PositionControl.cfg cfg/VelocityControl.cfg)

install(PROGRAMS scripts/acceleration_control.py
                 scripts/position_control.py 
                 scripts/velocity_control.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config cfg
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

# ament_export_dependencies(ament_cmake)
# ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
