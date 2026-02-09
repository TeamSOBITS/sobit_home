#include <iostream>
#include <random>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sobit_home_control/swerve_controller_control.hpp"
#include "sobit_home_control/swerve_controller_odometry.hpp"

namespace sobit_home
{
class SwerveController : public rclcpp::Node
{
public:
  explicit SwerveController(const rclcpp::NodeOptions & options);
  ~SwerveController();

  void play_sound(bool is_startup);

private:    
  // ROS2 I/F
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_vel_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_info_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          pub_odometry_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_steer_joint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheel_joint_;

  // Control & Sensing Callbacks
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_info);
  void control_callback();

  // Control Variables
  std_msgs::msg::Float64MultiArray wheel_joint_pos;
  std_msgs::msg::Float64MultiArray wheel_joint_vel;

  std::map<std::string, double> joints_pos;

  std::unique_ptr<SobitHomeControl>  sobit_home_control_;
  std::unique_ptr<SobitHomeOdometry> sobit_home_odometry_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::vector<std::string> steering_joints_names;
  std::vector<std::string> drive_joints_names;

  int CYCLE_FEQUENCY;
  double STEER_MAX_VEL;
  double DRIVING_STATUS_THRESHOLD; // Driving status threshold [rad]
};

} // namespace sobit_home

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::SwerveController)