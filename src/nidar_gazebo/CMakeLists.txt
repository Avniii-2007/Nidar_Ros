cmake_minimum_required(VERSION 3.8)
project(nidar_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/drone_autonomy_controller.py
    scripts/survivor_detector.py
    scripts/mission_planner_gcs.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
