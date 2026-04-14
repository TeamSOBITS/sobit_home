#ifndef SOBIT_HOME_LIBRARY__SOBIT_HOME_ARM_TELEOP_HPP_
#define SOBIT_HOME_LIBRARY__SOBIT_HOME_ARM_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

namespace sobit_home
{

struct ArmTeleopConfig {
  std::string planning_group;
  std::string target_frame;
  std::string base_frame;
  std::string trajectory_topic;
};

class MoveitArmTeleop : public rclcpp::Node
{
public:
  explicit MoveitArmTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MoveitArmTeleop();

private:
  struct ArmData {
    ArmTeleopConfig config;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub;
    std::atomic<bool> enabled{false};
    std::atomic<bool> thread_active{false};
    std::thread thread;

    // Cached TOTG joint limit maps — built once after MoveGroupInterface init.
    // Avoids rebuilding from parameters on every control cycle.
    std::unordered_map<std::string, double> vel_limits;
    std::unordered_map<std::string, double> accel_limits;

    ArmData() = default;
    ArmData(const ArmData &) = delete;
    ArmData & operator=(const ArmData &) = delete;
  };

  void init_move_groups();

  void enable_callback(
    const std::string & arm_name,
    const std_msgs::msg::Bool::SharedPtr msg);

  void tracking_loop(const std::string & arm_name);

  static double pose_distance(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b);

  std::thread init_thread_;

  std::unordered_map<std::string, std::unique_ptr<ArmData>> arms_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> enable_subs_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Global tuning parameters
  double update_rate_hz_;
  double max_cartesian_step_m_;
  double min_cartesian_fraction_;
  double arrival_threshold_m_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double eef_step_m_;              // Cartesian interpolation resolution [m]
  double replan_threshold_m_;      // Min target movement to trigger replan [m]
  int    traj_lookahead_ms_;       // Stamp offset added to trajectory header [ms]
  double ompl_planning_timeout_s_; // Max time for OMPL fallback plan [s]

  double last_heartbeat_sec_{0.0};
  static constexpr double heartbeat_period_sec_ = 2.0;
};

}  // namespace sobit_home

#endif  // SOBIT_HOME_LIBRARY__SOBIT_HOME_ARM_TELEOP_HPP_
