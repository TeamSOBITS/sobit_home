#ifndef SWERVE_CONTROLLER_CONTROL_HPP_
#define SWERVE_CONTROLLER_CONTROL_HPP_

#include <cmath>
#include <array>
#include <chrono>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/rclcpp.hpp>

class SobitHomeControl{
private:

  rclcpp::Node* node_;

public:
  enum MODE {
    NONE = -1,
    STOP_MOTION_MODE,
    TRANSLATIONAL_MOTION_MODE,
    ROTATIONAL_MOTION_MODE,
    SWIVEL_MOTION_MODE
  } motion_mode;

  double DRIVE_MAX_VEL;    // Driving moter's max speed [rad/s]
  double WHEEL_X_DISTANCE; // Wheel Distance between front and back [m]
  double WHEEL_Y_DISTANCE; // Wheel Distance between left and right [m]
  double WHEEL_RADIUS;     // Wheel Radius [m]

  double goal_steer_pos[4] = {0, };
  double goal_drive_vel[4] = {0, };
  double current_steer_pos[4] = {0, };

  // Constructor
  SobitHomeControl(rclcpp::Node* node) : node_(node) {
    DRIVE_MAX_VEL = node_->get_parameter("mobile_base.drive_max_vel").as_double();
    WHEEL_X_DISTANCE = node_->get_parameter("mobile_base.wheel_x_distance").as_double();
    WHEEL_Y_DISTANCE = node_->get_parameter("mobile_base.wheel_y_distance").as_double();
    WHEEL_RADIUS = node_->get_parameter("mobile_base.wheel_radius").as_double();
    RCLCPP_INFO(node_->get_logger(), "SOBIT HOME Wheel Control initialized.");
  }
  // Destructor
  ~SobitHomeControl() {
    RCLCPP_INFO(node_->get_logger(), "SOBIT HOME Wheel Control destroyed.");
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr vel_twist);
};

#endif // SWERVE_CONTROLLER_CONTROL_HPP_