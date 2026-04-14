cmake_minimum_required(VERSION 3.8)
project(sobit_home_control)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(position_controllers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

## Include directories
include_directories(include)


# Library
add_library(${PROJECT_NAME} SHARED
  src/swerve_controller_control.cpp
  src/swerve_controller_odometry.cpp
  src/swerve_controller_main.cpp
  src/moveit_whole_body_bridge.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_home::SwerveController"
  EXECUTABLE swerve_controller_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_home::MoveitWholeBodyBridge"
  EXECUTABLE moveit_whole_body_bridge_node
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  ament_index_cpp
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)


install(DIRECTORY
  config
  mp3
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
