#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/kinematic_solver_info.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace sobit_home_kinematics_plugin
{

class SobitHomeKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;

  const std::vector<std::string>& getJointNames() const override;
  const std::vector<std::string>& getLinkNames() const override;

  bool supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out = nullptr) const override;

private:
  bool timedOut(const rclcpp::Time& start_time, double duration) const;

  bool initialized_ = false;
  unsigned int dimension_;
  moveit_msgs::msg::KinematicSolverInfo solver_info_;

  const moveit::core::JointModelGroup* joint_model_group_ = nullptr;
  mutable moveit::core::RobotStatePtr state_;

  const moveit::core::JointModelGroup* arm_jmg_         = nullptr;
  const moveit::core::JointModelGroup* mobile_base_jmg_ = nullptr;
  const moveit::core::JointModel*      mobile_base_joint_ = nullptr;
  const moveit::core::JointModelGroup* body_jmg_        = nullptr;  // optional, nullptr if not present

  double optimal_arm_reach_ = 0.45; // Optimal reach of SOBIT HOME arm from base center [m]
};

}  // namespace sobit_home_kinematics_plugin
