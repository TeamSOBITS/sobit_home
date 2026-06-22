cmake_minimum_required(VERSION 3.14)
project(sobit_home_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)

# Find libraries
include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/sobit_home_joint_action_server.cpp
  src/sobit_home_wheel_action_server.cpp
  src/sobit_home_moveit_server.cpp
  src/kinematics.cpp
)

# Register Components
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_home::JointActionServer"
  EXECUTABLE joint_action_server)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_home::WheelActionServer"
  EXECUTABLE wheel_action_server)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_home::MoveitServer"
  EXECUTABLE moveit_action_server)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  "rclcpp_components"
  "sobits_interfaces"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "trajectory_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
  "tf2"
  "urdf"
  "moveit_ros_planning_interface"
  "moveit_core"
  "moveit_msgs"
)

#############
## Install ##
#############

install(DIRECTORY
  config
  launch DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()