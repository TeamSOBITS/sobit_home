#ifndef SWERVE_CONTROLLER_ODOMETRY_HPP_
#define SWERVE_CONTROLLER_ODOMETRY_HPP_

#include <cmath>
#include <memory>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/rclcpp.hpp>

// Wrap an angle (difference) into [-pi, pi]
static inline double wrap_pi(double a)
{
  while (a >  M_PI) a -= 2. * M_PI;
  while (a < -M_PI) a += 2. * M_PI;
  return a;
}

class SobitHomeOdometry{
private:
  rclcpp::Node* node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  int CYCLE_FEQUENCY;
  double WHEEL_X_DISTANCE; // Wheel Distance between front and back [m]
  double WHEEL_Y_DISTANCE; // Wheel Distance between left and right [m]
  double WHEEL_RADIUS;     // Wheel Radius [m]
  double STEER_SETTLE_VEL; // Steer speed above which a wheel is excluded from odometry [rad/s]

  double current_steer_pos[4] = {0, };
  double prev_steer_pos[4] = {0, };
  double current_drive_pos[4] = {0, };
  double prev_drive_pos[4] = {0, };

  nav_msgs::msg::Odometry odom_;

  // Constructor
  SobitHomeOdometry(rclcpp::Node* node) : node_(node) {
    CYCLE_FEQUENCY = node_->get_parameter("cycle_fequency").as_int();
    WHEEL_X_DISTANCE = node_->get_parameter("mobile_base.wheel_x_distance").as_double();
    WHEEL_Y_DISTANCE = node_->get_parameter("mobile_base.wheel_y_distance").as_double();
    WHEEL_RADIUS = node_->get_parameter("mobile_base.wheel_radius").as_double();
    STEER_SETTLE_VEL = node_->get_parameter("mobile_base.steer_settle_vel").as_double();
    RCLCPP_INFO(node_->get_logger(), "SOBIT HOME Odometry initialized.");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }
  ~SobitHomeOdometry() {
    RCLCPP_INFO(node_->get_logger(), "SOBIT HOME Odometry destroyed.");
  }

  void update_odom(double dt);

  double distance_calculation(double wheel_delta_pos);
  void pose_broadcaster();
};

#endif // SWERVE_CONTROLLER_ODOMETRY_HPP_