#include "sobit_home_control/moveit_whole_body_bridge.hpp"

#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

namespace sobit_home
{

MoveitWholeBodyBridge::MoveitWholeBodyBridge(const rclcpp::NodeOptions & options)
: Node("moveit_whole_body_bridge", options)
{
  this->declare_parameter("kp_xy", 1.5);
  this->declare_parameter("kp_theta", 2.0);
  this->declare_parameter("max_v_xy", 0.3);
  this->declare_parameter("max_omega", 1.0);

  kp_xy_    = this->get_parameter("kp_xy").as_double();
  kp_theta_ = this->get_parameter("kp_theta").as_double();
  max_v_xy_  = this->get_parameter("max_v_xy").as_double();
  max_omega_ = this->get_parameter("max_omega").as_double();

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&MoveitWholeBodyBridge::odom_callback, this, std::placeholders::_1));

  base_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this, "mobile_base_controller/follow_joint_trajectory",
    std::bind(&MoveitWholeBodyBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MoveitWholeBodyBridge::handle_cancel, this, std::placeholders::_1),
    std::bind(&MoveitWholeBodyBridge::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "MoveIt Mobile Base Bridge Component initialized.");
}

void MoveitWholeBodyBridge::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
}

double MoveitWholeBodyBridge::yaw_from_odom(const nav_msgs::msg::Odometry & odom)
{
  tf2::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

rclcpp_action::GoalResponse MoveitWholeBodyBridge::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveitWholeBodyBridge::handle_cancel(
  const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel base trajectory");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitWholeBodyBridge::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  std::thread{std::bind(&MoveitWholeBodyBridge::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void MoveitWholeBodyBridge::execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing base trajectory via cmd_vel...");
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  const auto goal = goal_handle->get_goal();
  const auto & points = goal->trajectory.points;

  if (points.empty()) {
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    return;
  }

  int idx_x = -1, idx_y = -1, idx_theta = -1;
  for (size_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
    const std::string & name = goal->trajectory.joint_names[i];
    if (name.find("virtual_base_joint/x") != std::string::npos) idx_x = i;
    else if (name.find("virtual_base_joint/y") != std::string::npos) idx_y = i;
    else if (name.find("virtual_base_joint/theta") != std::string::npos) idx_theta = i;
  }

  if (idx_x == -1 || idx_y == -1 || idx_theta == -1) {
    RCLCPP_WARN(this->get_logger(), "Base joint indices not found in trajectory, skipping.");
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    return;
  }

  auto start_time = this->now();

  for (size_t i = 0; i < points.size(); ++i) {
    if (goal_handle->is_canceling()) {
      publish_zero_twist();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      goal_handle->canceled(result);
      return;
    }

    const auto & pt = points[i];

    // Sleep until this waypoint's scheduled time using sim-time aware wait loop
    auto target_time = start_time + rclcpp::Duration(pt.time_from_start);
    while (this->now() < target_time) {
      if (goal_handle->is_canceling()) {
        publish_zero_twist();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    }

    // Desired pose from trajectory
    double des_x     = pt.positions[idx_x];
    double des_y     = pt.positions[idx_y];
    double des_theta = pt.positions[idx_theta];

    // Current pose from odometry
    nav_msgs::msg::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = latest_odom_;
    }
    double cur_x     = odom.pose.pose.position.x;
    double cur_y     = odom.pose.pose.position.y;
    double cur_theta = yaw_from_odom(odom);

    // Pose error in world frame
    double err_x     = des_x - cur_x;
    double err_y     = des_y - cur_y;
    double err_theta = des_theta - cur_theta;

    // Wrap angle error to [-π, π]
    while (err_theta >  M_PI) err_theta -= 2.0 * M_PI;
    while (err_theta < -M_PI) err_theta += 2.0 * M_PI;

    // Feedforward from finite differences
    double ff_vx = 0.0, ff_vy = 0.0, ff_omega = 0.0;
    if (i + 1 < points.size()) {
      const auto & next = points[i + 1];
      double dt = rclcpp::Duration(next.time_from_start).seconds() -
                  rclcpp::Duration(pt.time_from_start).seconds();
      if (dt > 1e-6) {
        ff_vx    = (next.positions[idx_x]     - pt.positions[idx_x])     / dt;
        ff_vy    = (next.positions[idx_y]     - pt.positions[idx_y])     / dt;
        ff_omega = (next.positions[idx_theta] - pt.positions[idx_theta]) / dt;
      }
    }

    // P feedback on position error
    double v_x_global = ff_vx    + kp_xy_    * err_x;
    double v_y_global = ff_vy    + kp_xy_    * err_y;
    double omega      = ff_omega + kp_theta_ * err_theta;

    // Clamp velocities
    double v_norm = std::sqrt(v_x_global * v_x_global + v_y_global * v_y_global);
    if (v_norm > max_v_xy_) {
      v_x_global *= max_v_xy_ / v_norm;
      v_y_global *= max_v_xy_ / v_norm;
    }
    omega = std::clamp(omega, -max_omega_, max_omega_);

    // Rotate global velocity into robot-local frame (for swerve drive)
    double v_x_local =  v_x_global * std::cos(cur_theta) + v_y_global * std::sin(cur_theta);
    double v_y_local = -v_x_global * std::sin(cur_theta) + v_y_global * std::cos(cur_theta);

    RCLCPP_DEBUG(this->get_logger(),
      "pt[%zu] des(%.3f,%.3f,%.3f) cur(%.3f,%.3f,%.3f) err(%.3f,%.3f,%.3f) cmd(%.3f,%.3f,%.3f)",
      i, des_x, des_y, des_theta, cur_x, cur_y, cur_theta,
      err_x, err_y, err_theta, v_x_local, v_y_local, omega);

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = v_x_local;
    twist.linear.y  = v_y_local;
    twist.angular.z = omega;
    cmd_vel_pub_->publish(twist);
  }

  publish_zero_twist();

  if (rclcpp::ok()) {
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Base trajectory succeeded.");
  }
}

void MoveitWholeBodyBridge::publish_zero_twist()
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;
  cmd_vel_pub_->publish(twist);
}

}  // namespace sobit_home

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::MoveitWholeBodyBridge)
