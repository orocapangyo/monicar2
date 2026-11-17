################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.14)
project(monicar2_localization)

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
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Build (C++ nodes, converted from Python) - targets for ROS 2 Jazzy
################################################################################
add_executable(odomPublisher src/odomPublisher.cpp)
target_link_libraries(odomPublisher PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2_ros::tf2_ros)

add_executable(rviz2ClickTo2d src/rviz2ClickTo2d.cpp)
target_link_libraries(rviz2ClickTo2d PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS})

add_executable(rviz2ClickTo2dTimer src/rviz2ClickTo2d.cpp)
target_link_libraries(rviz2ClickTo2dTimer PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS})
target_compile_definitions(rviz2ClickTo2dTimer PRIVATE USE_TIMER)

add_executable(move src/move_rotate.cpp)
target_link_libraries(move PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS})

add_executable(rotate src/move_rotate.cpp)
target_link_libraries(rotate PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS})
target_compile_definitions(rotate PRIVATE ROTATE)


install(TARGETS
  odomPublisher
  rviz2ClickTo2d
  rviz2ClickTo2dTimer
  move
  rotate
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
