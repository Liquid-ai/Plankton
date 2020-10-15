cmake_minimum_required(VERSION 3.5)
project(uuv_tutorial_rov_model)

find_package(ament_cmake REQUIRED)

set(PACKAGE_NAME tutorial_rov_model)

install(DIRECTORY mesh robots urdf launch
  DESTINATION share/${PROJECT_NAME}/models/${PACKAGE_NAME}
  PATTERN "*~" EXCLUDE)

ament_package()
