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

  // --- Identify required sub-groups ---

  // Mobile base: sub-group containing a single PLANAR joint
  // (also accept the group itself having the PLANAR joint directly — no sub-groups case)
  auto is_planar_group = [](const moveit::core::JointModelGroup* g) {
    const auto& joints = g->getJointModels();
    return joints.size() == 1 && joints[0]->getType() == moveit::core::JointModel::PLANAR;
  };

  auto base_it = std::find_if(sub_groups.cbegin(), sub_groups.cend(), is_planar_group);
  if (base_it == sub_groups.cend()) {
    // Check if the group itself is the planar base (no sub-groups)
    if (is_planar_group(joint_model_group_)) {
      RCLCPP_WARN(LOGGER, "Group '%s' is a standalone mobile base — no IK to solve.", group_name.c_str());
      // Still initialize minimally so we don't crash, but nothing useful to do
      dimension_ = joint_model_group_->getVariableCount();
      solver_info_.link_names.push_back(base_frame);
      state_.reset(new moveit::core::RobotState(robot_model_));
      mobile_base_jmg_  = joint_model_group_;
      mobile_base_joint_ = joint_model_group_->getJointModels()[0];
      initialized_ = true;
      return true;
    }
    RCLCPP_ERROR(LOGGER, "Group '%s': failed to find mobile base sub-group (single PLANAR joint)", group_name.c_str());
    return false;
  }
  mobile_base_jmg_   = *base_it;
  mobile_base_joint_ = mobile_base_jmg_->getJointModels()[0];

  // Arm: chain sub-group with the most active joints (distinguishes arm from single-joint body)
  arm_jmg_ = nullptr;
  for (const auto* sg : sub_groups) {
    if (sg == mobile_base_jmg_) continue;
    if (sg->isChain()) {
      if (!arm_jmg_ || sg->getActiveJointModels().size() > arm_jmg_->getActiveJointModels().size())
        arm_jmg_ = sg;
    }
  }
  if (!arm_jmg_) {
    RCLCPP_ERROR(LOGGER, "Group '%s': failed to find arm sub-group (chain)", group_name.c_str());
    return false;
  }

  // Body (optional): first remaining sub-group that has at least one PRISMATIC or REVOLUTE active joint
  body_jmg_ = nullptr;
  for (const auto* sg : sub_groups) {
    if (sg == mobile_base_jmg_ || sg == arm_jmg_) continue;
    for (const auto* j : sg->getActiveJointModels()) {
      auto type = j->getType();
      if (type == moveit::core::JointModel::PRISMATIC || type == moveit::core::JointModel::REVOLUTE) {
        body_jmg_ = sg;
        break;
      }
    }
    if (body_jmg_) break;
  }

  // --- Build solver info (joint names + limits) ---
  for (const auto& joint : joint_model_group_->getActiveJointModels()) {
    solver_info_.joint_names.push_back(joint->getName());
    const auto& bounds = joint->getVariableBoundsMsg();
    solver_info_.limits.insert(solver_info_.limits.end(), bounds.begin(), bounds.end());
  }

  // --- Find variable indices in the group state vector ---
  auto var_names = joint_model_group_->getVariableNames();

  auto base_var_names = mobile_base_jmg_->getVariableNames();
  auto base_it2 = std::find(var_names.begin(), var_names.end(), base_var_names[0]);
  mobile_base_index_ = std::distance(var_names.begin(), base_it2);

  if (body_jmg_) {
    auto body_var_names = body_jmg_->getVariableNames();
    auto body_it = std::find(var_names.begin(), var_names.end(), body_var_names[0]);
    body_index_ = std::distance(var_names.begin(), body_it);
  }

  dimension_ = joint_model_group_->getVariableCount();
  solver_info_.link_names.push_back(getTipFrame());
  state_.reset(new moveit::core::RobotState(robot_model_));

  initialized_ = true;
  RCLCPP_INFO(LOGGER, "SOBIT HOME Whole-Body Kinematics Plugin initialized for group '%s' (body sub-group: %s).",
              group_name.c_str(), body_jmg_ ? body_jmg_->getName().c_str() : "none");
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
  if (!initialized_) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  solution = ik_seed_state;

  // Extract seed base pose
  double base_x   = ik_seed_state[mobile_base_index_];
  double base_y   = ik_seed_state[mobile_base_index_ + 1];
  double base_yaw = ik_seed_state[mobile_base_index_ + 2];

  Eigen::Isometry3d target_pose;
  tf2::fromMsg(ik_pose, target_pose);

  double target_x = target_pose.translation().x();
  double target_y = target_pose.translation().y();

  // --- BASE HEURISTIC: position base at optimal reach distance from target ---
  double dx   = target_x - base_x;
  double dy   = target_y - base_y;
  double dist = std::sqrt(dx*dx + dy*dy);
  if (dist > optimal_arm_reach_ + 0.05 || dist < optimal_arm_reach_ - 0.05) {
    double angle_to_target = std::atan2(dy, dx);
    base_x   = target_x - optimal_arm_reach_ * std::cos(angle_to_target);
    base_y   = target_y - optimal_arm_reach_ * std::sin(angle_to_target);
    base_yaw = angle_to_target;
  }
  solution[mobile_base_index_]     = base_x;
  solution[mobile_base_index_ + 1] = base_y;
  solution[mobile_base_index_ + 2] = base_yaw;

  // Apply base position to robot state
  state_->setJointGroupPositions(mobile_base_jmg_, &solution[mobile_base_index_]);
  state_->updateLinkTransforms();

  // --- BODY HEURISTIC (if present): set lift height to match target z ---
  if (body_jmg_) {
    const auto body_ik_solver = body_jmg_->getGroupKinematics().first.solver_instance_;
    if (body_ik_solver) {
      // Use body sub-solver to find lift height; pass z-only pose
      double target_z = target_pose.translation().z();
      const auto& body_bounds = body_jmg_->getJointModels()[0]->getVariableBounds()[0];
      double body_height = std::clamp(target_z, body_bounds.min_position_, body_bounds.max_position_);
      solution[body_index_] = body_height;
      state_->setJointGroupPositions(body_jmg_, &solution[body_index_]);
      state_->updateLinkTransforms();
    }
  }

  // --- ARM IK: solve in arm's local frame ---
  const auto arm_ik_solver = arm_jmg_->getGroupKinematics().first.solver_instance_;
  if (!arm_ik_solver) {
    RCLCPP_ERROR(LOGGER, "No IK solver found for arm sub-group '%s'", arm_jmg_->getName().c_str());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  // Build arm-only seed state (strip base and body variables)
  std::vector<double> arm_seed;
  for (size_t i = 0; i < ik_seed_state.size(); ++i) {
    bool is_base = (i >= mobile_base_index_ && i <= mobile_base_index_ + 2);
    bool is_body = body_jmg_ && (i == body_index_);
    if (!is_base && !is_body)
      arm_seed.push_back(ik_seed_state[i]);
  }

  // Transform target into arm's base frame
  Eigen::Isometry3d arm_base_transform = state_->getFrameTransform(arm_ik_solver->getBaseFrame());
  geometry_msgs::msg::Pose arm_local_pose = tf2::toMsg(arm_base_transform.inverse() * target_pose);

  // Wrap callback to re-embed arm solution into full solution
  IKCallbackFn arm_callback;
  if (solution_callback) {
    arm_callback = [&solution_callback, &solution, this](
        const geometry_msgs::msg::Pose& pose, const std::vector<double>& arm_sol,
        moveit_msgs::msg::MoveItErrorCodes& err) {
      std::vector<double> full_sol = solution;
      size_t arm_idx = 0;
      for (size_t i = 0; i < full_sol.size(); ++i) {
        bool is_base = (i >= mobile_base_index_ && i <= mobile_base_index_ + 2);
        bool is_body = body_jmg_ && (i == body_index_);
        if (!is_base && !is_body)
          full_sol[i] = arm_sol[arm_idx++];
      }
      solution_callback(pose, full_sol, err);
    };
  }

  std::vector<double> arm_solution;
  bool ik_valid = arm_ik_solver->searchPositionIK(arm_local_pose, arm_seed, timeout,
                                                  consistency_limits, arm_solution,
                                                  arm_callback, error_code, options);
  if (ik_valid) {
    size_t arm_idx = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      bool is_base = (i >= mobile_base_index_ && i <= mobile_base_index_ + 2);
      bool is_body = body_jmg_ && (i == body_index_);
      if (!is_base && !is_body)
        solution[i] = arm_solution[arm_idx++];
    }
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

// --- Overload forwarders ---

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
  for (size_t i = 0; i < poses.size(); i++)
    poses[i] = tf2::toMsg(state_->getFrameTransform(link_names[i]));
  return true;
}

const std::vector<std::string>& SobitHomeKinematicsPlugin::getJointNames() const { return solver_info_.joint_names; }
const std::vector<std::string>& SobitHomeKinematicsPlugin::getLinkNames() const { return solver_info_.link_names; }

bool SobitHomeKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const
{
  auto is_planar_group = [](const moveit::core::JointModelGroup* g) {
    const auto& joints = g->getJointModels();
    return joints.size() == 1 && joints[0]->getType() == moveit::core::JointModel::PLANAR;
  };

  // Accept standalone mobile base group (no sub-groups, just the planar joint)
  if (is_planar_group(jmg)) return true;

  std::vector<const moveit::core::JointModelGroup*> sub_groups;
  jmg->getSubgroups(sub_groups);

  // Require a PLANAR sub-group (mobile base)
  const moveit::core::JointModelGroup* base_sg = nullptr;
  for (const auto* sg : sub_groups) {
    if (is_planar_group(sg)) { base_sg = sg; break; }
  }
  if (!base_sg) {
    if (error_text_out)
      *error_text_out = "SobitHomeKinematicsPlugin requires a sub-group with a single PLANAR joint (mobile base)";
    return false;
  }

  // Arm: the chain sub-group with the most joints (excludes single-joint body groups)
  const moveit::core::JointModelGroup* arm_sg = nullptr;
  for (const auto* sg : sub_groups) {
    if (sg == base_sg) continue;
    if (sg->isChain()) {
      if (!arm_sg || sg->getActiveJointModels().size() > arm_sg->getActiveJointModels().size())
        arm_sg = sg;
    }
  }
  if (!arm_sg) {
    if (error_text_out)
      *error_text_out = "SobitHomeKinematicsPlugin requires a chain sub-group (arm)";
    return false;
  }

  // Remaining sub-groups are accepted as optional body/lift groups (e.g. body_lift_joint)
  // We don't restrict their structure here — initialize() will detect them by active joint type.
  return true;
}

}  // namespace sobit_home_kinematics_plugin

// Register as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(sobit_home_kinematics_plugin::SobitHomeKinematicsPlugin, kinematics::KinematicsBase)
