################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(monicar2_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Build (C++ nodes, converted from Python)
################################################################################
add_executable(teleop_joy src/teleop_joy.cpp)
ament_target_dependencies(teleop_joy rclcpp geometry_msgs sensor_msgs std_msgs)

add_executable(teleop_keyboard src/teleop_keyboard.cpp)
ament_target_dependencies(teleop_keyboard rclcpp geometry_msgs std_msgs)


install(TARGETS
  teleop_joy
  teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
