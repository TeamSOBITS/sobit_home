#ifndef SOBIT_HOME_CONTROL_MOVEIT_WHOLE_BODY_BRIDGE_HPP_
#define SOBIT_HOME_CONTROL_MOVEIT_WHOLE_BODY_BRIDGE_HPP_

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace sobit_home
{

class MoveitWholeBodyBridge : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  explicit MoveitWholeBodyBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MoveitWholeBodyBridge() override = default;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr base_server_;

  // Latest odometry, protected by mutex (odom arrives on a separate thread)
  nav_msgs::msg::Odometry latest_odom_;
  std::mutex odom_mutex_;

  // P-gain for pose error → velocity correction
  double kp_xy_    = 0.0;
  double kp_theta_ = 0.0;

  // Velocity limits
  double max_v_xy_  = 0.0;
  double max_omega_ = 0.0;

  void execute(const std::shared_ptr<GoalHandleFJT> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void publish_zero_twist();

  // Returns yaw from a quaternion msg
  static double yaw_from_odom(const nav_msgs::msg::Odometry & odom);
};

}  // namespace sobit_home

#endif  // SOBIT_HOME_CONTROL_MOVEIT_WHOLE_BODY_BRIDGE_HPP_
