cmake_minimum_required(VERSION 3.5.1)
project(haptiquad_rosbag_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts
DESTINATION share/${PROJECT_NAME})

ament_python_install_package(scripts)

install(
  PROGRAMS  scripts/msg_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
