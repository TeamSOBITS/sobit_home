#include "sobit_home_kinematics_plugin/sobit_home_kinematics_plugin.hpp"
#include <class_loader/class_loader.hpp>

namespace sobit_home_kinematics_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("sobit_home_kinematics_plugin");

bool SobitHomeKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                                           const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                           const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                           double search_discretization)
{
  node_ = node;
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_) return false;

  std::vector<const moveit::core::JointModelGroup*> sub_groups;
  joint_model_group_->getSubgroups(sub_groups);

  if (sub_groups.size() != 2) {
    RCLCPP_ERROR(LOGGER, "Group '%s' must have exactly two sub-groups (arm and mobile_base)", group_name.c_str());
    return false;
  }

  // 1. Identify the Arm subgroup
  auto arm_it = std::find_if(sub_groups.cbegin(), sub_groups.cend(), [](const moveit::core::JointModelGroup* const group) {
    return group->isChain(); // Arms are usually chains
  });
  if (arm_it != sub_groups.cend()) {
    arm_jmg_ = *arm_it;
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to find arm sub-group");
    return false;
  }

  // 2. Identify the Mobile Base subgroup (Planar joint)
  auto base_it = std::find_if(sub_groups.cbegin(), sub_groups.cend(), [](const moveit::core::JointModelGroup* const group) {
    return group->getJointModels().size() == 1 && group->getJointModels()[0]->getType() == moveit::core::JointModel::PLANAR;
  });
  if (base_it != sub_groups.cend()) {
    mobile_base_jmg_ = *base_it;
    mobile_base_joint_ = mobile_base_jmg_->getJointModels()[0];
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to find mobile base sub-group");
    return false;
  }

  // Determine indices
  // The variables order is usually [arm_joints..., base_x, base_y, base_theta] or vice versa.
  // We extract names and limits.
  for (const auto& joint : joint_model_group_->getActiveJointModels()) {
    solver_info_.joint_names.push_back(joint->getName());
    const auto& bounds = joint->getVariableBoundsMsg();
    solver_info_.limits.insert(solver_info_.limits.end(), bounds.begin(), bounds.end());
  }

  // Find where the mobile base variables start in the combined state vector
  auto var_names = joint_model_group_->getVariableNames();
  auto base_var_names = mobile_base_jmg_->getVariableNames();
  auto it = std::find(var_names.begin(), var_names.end(), base_var_names[0]);
  mobile_base_index_ = std::distance(var_names.begin(), it);

  dimension_ = joint_model_group_->getVariableCount();

  solver_info_.link_names.push_back(getTipFrame());
  state_.reset(new moveit::core::RobotState(robot_model_));

  initialized_ = true;
  RCLCPP_INFO(LOGGER, "SOBIT HOME Whole-Body Kinematics Plugin initialized.");
  return true;
}

bool SobitHomeKinematicsPlugin::timedOut(const rclcpp::Time& start_time, double duration) const {
  return ((node_->now() - start_time).seconds() >= duration);
}

bool SobitHomeKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  rclcpp::Time start_time = node_->now();
  if (!initialized_) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  solution = ik_seed_state;

  // Split seed state
  std::vector<double> arm_jmg_ik_seed_state;
  double base_x = ik_seed_state[mobile_base_index_];
  double base_y = ik_seed_state[mobile_base_index_ + 1];
  double base_yaw = ik_seed_state[mobile_base_index_ + 2];

  for(size_t i = 0; i < ik_seed_state.size(); ++i) {
      if(i < mobile_base_index_ || i > mobile_base_index_ + 2) {
          arm_jmg_ik_seed_state.push_back(ik_seed_state[i]);
      }
  }

  Eigen::Isometry3d target_pose;
  tf2::fromMsg(ik_pose, target_pose);

  // --- BASE HEURISTIC ---
  // Target position in world/odom frame
  double target_x = target_pose.translation().x();
  double target_y = target_pose.translation().y();

  // Distance from current base to target
  double dx = target_x - base_x;
  double dy = target_y - base_y;
  double dist = std::sqrt(dx*dx + dy*dy);

  // If the target is too far or too close, move the base
  if (dist > optimal_arm_reach_ + 0.05 || dist < optimal_arm_reach_ - 0.05) {
      // Calculate new base position placing target at exactly optimal_arm_reach_
      double angle_to_target = std::atan2(dy, dx);
      base_x = target_x - optimal_arm_reach_ * std::cos(angle_to_target);
      base_y = target_y - optimal_arm_reach_ * std::sin(angle_to_target);
      base_yaw = angle_to_target; // Face the target
  }

  solution[mobile_base_index_] = base_x;
  solution[mobile_base_index_ + 1] = base_y;
  solution[mobile_base_index_ + 2] = base_yaw;

  // Update Robot State with new base position
  state_->setJointGroupPositions(mobile_base_jmg_, &solution[mobile_base_index_]);
  state_->updateLinkTransforms();

  // --- ARM IK ---
  const auto arm_ik_solver = arm_jmg_->getGroupKinematics().first.solver_instance_;
  
  // Transform target pose from global frame to arm's base frame (usually base_footprint)
  Eigen::Isometry3d arm_base_transform = state_->getFrameTransform(arm_ik_solver->getBaseFrame());
  geometry_msgs::msg::Pose arm_local_target_pose = tf2::toMsg(arm_base_transform.inverse() * target_pose);

  std::vector<double> arm_solution;
  
  // Create a wrapper callback
  IKCallbackFn arm_solution_callback;
  if (solution_callback) {
      arm_solution_callback = [&solution_callback, &solution, this](const geometry_msgs::msg::Pose& pose, const std::vector<double>& arm_sol, moveit_msgs::msg::MoveItErrorCodes& err) {
          std::vector<double> full_sol = solution;
          size_t arm_idx = 0;
          for(size_t i = 0; i < full_sol.size(); ++i) {
              if(i < mobile_base_index_ || i > mobile_base_index_ + 2) {
                  full_sol[i] = arm_sol[arm_idx++];
              }
          }
          solution_callback(pose, full_sol, err);
      };
  }

  bool ik_valid = arm_ik_solver->searchPositionIK(arm_local_target_pose, arm_jmg_ik_seed_state, timeout,
                                                  consistency_limits, arm_solution, arm_solution_callback, error_code, options);

  if (ik_valid) {
      size_t arm_idx = 0;
      for(size_t i = 0; i < solution.size(); ++i) {
          if(i < mobile_base_index_ || i > mobile_base_index_ + 2) {
              solution[i] = arm_solution[arm_idx++];
          }
      }
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      return true;
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

// Implement standard required overrides passing them to the main searchPositionIK
bool SobitHomeKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                              std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool SobitHomeKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool SobitHomeKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                                 moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const {
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool SobitHomeKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code, options);
}

bool SobitHomeKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                              std::vector<geometry_msgs::msg::Pose>& poses) const {
  if (!initialized_) return false;
  poses.resize(link_names.size());
  state_->setJointGroupPositions(joint_model_group_, joint_angles);
  state_->updateLinkTransforms();
  for (size_t i = 0; i < poses.size(); i++) {
    poses[i] = tf2::toMsg(state_->getFrameTransform(link_names[i]));
  }
  return true;
}

const std::vector<std::string>& SobitHomeKinematicsPlugin::getJointNames() const { return solver_info_.joint_names; }
const std::vector<std::string>& SobitHomeKinematicsPlugin::getLinkNames() const { return solver_info_.link_names; }

bool SobitHomeKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const
{
  // Accept any group that has exactly two sub-groups: one chain (arm) and one planar (mobile base)
  std::vector<const moveit::core::JointModelGroup*> sub_groups;
  jmg->getSubgroups(sub_groups);
  if (sub_groups.size() != 2) {
    if (error_text_out)
      *error_text_out = "SobitHomeKinematicsPlugin requires a group with exactly two sub-groups (arm chain + mobile base)";
    return false;
  }
  bool has_chain = std::any_of(sub_groups.begin(), sub_groups.end(),
                               [](const moveit::core::JointModelGroup* g) { return g->isChain(); });
  bool has_planar = std::any_of(sub_groups.begin(), sub_groups.end(),
                                [](const moveit::core::JointModelGroup* g) {
                                  return g->getJointModels().size() == 1 &&
                                         g->getJointModels()[0]->getType() == moveit::core::JointModel::PLANAR;
                                });
  if (!has_chain || !has_planar) {
    if (error_text_out)
      *error_text_out = "SobitHomeKinematicsPlugin requires one chain sub-group (arm) and one planar sub-group (mobile base)";
    return false;
  }
  return true;
}

}  // namespace sobit_home_kinematics_plugin

// Register as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(sobit_home_kinematics_plugin::SobitHomeKinematicsPlugin, kinematics::KinematicsBase)
