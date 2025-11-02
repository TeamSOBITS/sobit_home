#include "sobit_home_library/sobit_home_joint_action_server.hpp"

namespace sobit_home{

JointActionServer::JointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("joint_action_server", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  this->action_server_move_joints_ = rclcpp_action::create_server<MoveJoint>(
      this,
      "move_joint",
      std::bind(&JointActionServer::handle_move_joints_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_joints_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_joints_accepted, this, std::placeholders::_1));
  this->action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&JointActionServer::handle_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_to_pose_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_to_pose_accepted, this, std::placeholders::_1));


  // default grasp mode
  this->service_server_move_hand_to_coord_left_ = this->create_service<MoveHandToTargetCoord>(
      "move_hand_to_coord/left",
      [this](const std::shared_ptr<MoveHandToTargetCoord::Request> request, std::shared_ptr<MoveHandToTargetCoord::Response> response) {
        serve_move_hand_to_coord(request, response, false, false);  // when target hand is left, 3rd arg is 'false'
      });
  this->service_server_move_hand_to_tf_left_ = this->create_service<MoveHandToTargetTF>(
      "move_hand_to_tf/left",
      [this](const std::shared_ptr<MoveHandToTargetTF::Request> request, std::shared_ptr<MoveHandToTargetTF::Response> response) {
        serve_move_hand_to_tf(request, response, false, false);  // when target hand is left, 3rd arg is 'false'
      });
  this->service_server_move_hand_to_coord_right_ = this->create_service<MoveHandToTargetCoord>(
      "move_hand_to_coord/right",
      [this](const std::shared_ptr<MoveHandToTargetCoord::Request> request, std::shared_ptr<MoveHandToTargetCoord::Response> response) {
        serve_move_hand_to_coord(request, response, true, false);  // when target hand is right, 3rd arg is 'true'
      });
  this->service_server_move_hand_to_tf_right_ = this->create_service<MoveHandToTargetTF>(
      "move_hand_to_tf/right",
      [this](const std::shared_ptr<MoveHandToTargetTF::Request> request, std::shared_ptr<MoveHandToTargetTF::Response> response) {
        serve_move_hand_to_tf(request, response, true, false);  // when target hand is right, 3rd arg is 'true'
      });

  // one link grasp mode
  this->service_server_move_hand_to_coord_one_left_ = this->create_service<MoveHandToTargetCoord>(
      "move_hand_to_coord/one_link/left",
      [this](const std::shared_ptr<MoveHandToTargetCoord::Request> request, std::shared_ptr<MoveHandToTargetCoord::Response> response) {
        serve_move_hand_to_coord(request, response, false, true);  // when target hand is left, 3rd arg is 'false'
      });
  this->service_server_move_hand_to_tf_one_left_ = this->create_service<MoveHandToTargetTF>(
      "move_hand_to_tf/one_link/left",
      [this](const std::shared_ptr<MoveHandToTargetTF::Request> request, std::shared_ptr<MoveHandToTargetTF::Response> response) {
        serve_move_hand_to_tf(request, response, false, true);  // when target hand is left, 3rd arg is 'false'
      });
  this->service_server_move_hand_to_coord_one_right_ = this->create_service<MoveHandToTargetCoord>(
      "move_hand_to_coord/one_link/right",
      [this](const std::shared_ptr<MoveHandToTargetCoord::Request> request, std::shared_ptr<MoveHandToTargetCoord::Response> response) {
        serve_move_hand_to_coord(request, response, true, true);  // when target hand is right, 3rd arg is 'true'
      });
  this->service_server_move_hand_to_tf_one_right_ = this->create_service<MoveHandToTargetTF>(
      "move_hand_to_tf/one_link/right",
      [this](const std::shared_ptr<MoveHandToTargetTF::Request> request, std::shared_ptr<MoveHandToTargetTF::Response> response) {
        serve_move_hand_to_tf(request, response, true, true);  // when target hand is right, 3rd arg is 'true'
      });


  this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&JointActionServer::joint_state_callback, this, std::placeholders::_1));
  this->pub_left_arm_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_left_position_controller/joint_trajectory", qos_profile);
  this->pub_left_hand_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "hand_left_position_controller/joint_trajectory", qos_profile);
  this->pub_right_arm_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_right_position_controller/joint_trajectory", qos_profile);
  this->pub_right_hand_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "hand_right_position_controller/joint_trajectory", qos_profile);
  this->pub_body_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "body_position_controller/joint_trajectory", qos_profile);
  this->pub_head_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "head_position_controller/joint_trajectory", qos_profile);

  //Declare the pose parameters
  this->declare_parameter("poses", std::vector<std::string>());
  auto pose_names = this->get_parameter("poses").as_string_array();

  poses_.clear();
  for (auto pose_name : pose_names) {
    // Declare parameters for each pose
    this->declare_parameter(pose_name + ".arm_right_shoulder_tilt", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_right_upper_roll", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_right_upper_flex", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_right_elbow", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_right_wrist_tilt", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_right_wrist_roll", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_right_finger_l_mcp", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_right_finger_l_dip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_right_finger_l_pip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_right_finger_c_mcp", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_right_finger_c_ip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_right_finger_r_dip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_right_finger_r_pip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_left_shoulder_tilt", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_left_upper_roll", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_left_upper_flex", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_left_elbow", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_left_wrist_tilt", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_left_wrist_roll", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_left_finger_l_mcp", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_left_finger_l_dip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_left_finger_l_pip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_left_finger_c_mcp", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_left_finger_c_ip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_left_finger_r_dip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand_left_finger_r_pip", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".body_lift", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_pan", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_tilt", rclcpp::PARAMETER_DOUBLE);

    // Read parameters for each pose
    PoseParams params;
    params.pose_name           = pose_name;
    params.arm_right_shoulder_tilt  = this->get_parameter(pose_name + ".arm_right_shoulder_tilt").as_double();
    params.arm_right_upper_roll     = this->get_parameter(pose_name + ".arm_right_upper_roll").as_double();
    params.arm_right_upper_flex     = this->get_parameter(pose_name + ".arm_right_upper_flex").as_double();
    params.arm_right_elbow          = this->get_parameter(pose_name + ".arm_right_elbow").as_double();
    params.arm_right_wrist_tilt     = this->get_parameter(pose_name + ".arm_right_wrist_tilt").as_double();
    params.arm_right_wrist_roll     = this->get_parameter(pose_name + ".arm_right_wrist_roll").as_double();
    params.hand_right_finger_l_mcp  = this->get_parameter(pose_name + ".hand_right_finger_l_mcp").as_double();
    params.hand_right_finger_l_dip  = this->get_parameter(pose_name + ".hand_right_finger_l_dip").as_double();
    params.hand_right_finger_l_pip  = this->get_parameter(pose_name + ".hand_right_finger_l_pip").as_double();
    params.hand_right_finger_c_mcp  = this->get_parameter(pose_name + ".hand_right_finger_c_mcp").as_double();
    params.hand_right_finger_c_ip   = this->get_parameter(pose_name + ".hand_right_finger_c_ip").as_double();
    params.hand_right_finger_r_dip  = this->get_parameter(pose_name + ".hand_right_finger_r_dip").as_double();
    params.hand_right_finger_r_pip  = this->get_parameter(pose_name + ".hand_right_finger_r_pip").as_double();
    params.arm_left_shoulder_tilt   = this->get_parameter(pose_name + ".arm_left_shoulder_tilt").as_double();
    params.arm_left_upper_roll      = this->get_parameter(pose_name + ".arm_left_upper_roll").as_double();
    params.arm_left_upper_flex      = this->get_parameter(pose_name + ".arm_left_upper_flex").as_double();
    params.arm_left_elbow           = this->get_parameter(pose_name + ".arm_left_elbow").as_double();
    params.arm_left_wrist_tilt      = this->get_parameter(pose_name + ".arm_left_wrist_tilt").as_double();
    params.arm_left_wrist_roll      = this->get_parameter(pose_name + ".arm_left_wrist_roll").as_double();
    params.hand_left_finger_l_mcp   = this->get_parameter(pose_name + ".hand_left_finger_l_mcp").as_double();
    params.hand_left_finger_l_dip   = this->get_parameter(pose_name + ".hand_left_finger_l_dip").as_double();
    params.hand_left_finger_l_pip   = this->get_parameter(pose_name + ".hand_left_finger_l_pip").as_double();
    params.hand_left_finger_c_mcp   = this->get_parameter(pose_name + ".hand_left_finger_c_mcp").as_double();
    params.hand_left_finger_c_ip    = this->get_parameter(pose_name + ".hand_left_finger_c_ip").as_double();
    params.hand_left_finger_r_dip   = this->get_parameter(pose_name + ".hand_left_finger_r_dip").as_double();
    params.hand_left_finger_r_pip   = this->get_parameter(pose_name + ".hand_left_finger_r_pip").as_double();
    params.body_lift                = this->get_parameter(pose_name + ".body_lift").as_double();
    params.head_pan                 = this->get_parameter(pose_name + ".head_pan").as_double();
    params.head_tilt                = this->get_parameter(pose_name + ".head_tilt").as_double();

    poses_.push_back(params);
  }

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been initialized.");
}
JointActionServer::~JointActionServer()
{
  this->action_server_move_joints_.reset();
  this->action_server_move_to_pose_.reset();

  this->sub_joint_state_.reset();
  this->pub_left_arm_joint_control_.reset();
  this->pub_left_hand_joint_control_.reset();
  this->pub_right_arm_joint_control_.reset();
  this->pub_right_hand_joint_control_.reset();
  this->pub_body_joint_control_.reset();
  this->pub_head_joint_control_.reset();

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been terminated.");
}


rclcpp_action::GoalResponse JointActionServer::handle_move_joints_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse JointActionServer::handle_move_to_pose_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse JointActionServer::handle_move_joints_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointActionServer::handle_move_to_pose_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void JointActionServer::handle_move_joints_accepted(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_joints, this, std::placeholders::_1), goal_handle}.detach();
}

void JointActionServer::handle_move_to_pose_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
}


void JointActionServer::exe_move_joints(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoint::Result>();

  // Check if the number of joint names and joint rad are the same
  if (goal->target_joint_names.size() != goal->target_joint_rad.size()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal request. The number of joint names and joint rad are different");
    result->success = false;
    result->message = "Invalid goal request. The number of joint names and joint rad are different";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Check if the joint names are valid
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    if (std::find(JointNames.begin(), JointNames.end(), goal->target_joint_names[i]) == JointNames.end()) {
      RCLCPP_ERROR(this->get_logger(), "The joint name does not exist: %s", goal->target_joint_names[i].c_str());
      result->success = false;
      result->message = "The joint name does not exist: " + goal->target_joint_names[i];
      result->total_elapsed_time.sec = 0;
      result->total_elapsed_time.nanosec = 0;
      goal_handle->abort(result);
      return;
    }
  }

  // TODO: Check if the joint rad are within the joint limits

  // Publish the joint trajectory
  std::vector<trajectory_msgs::msg::JointTrajectory> joint_trajectory;
  joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance);

  try {
    this->pub_left_arm_joint_control_->publish(joint_trajectory[0]);
    this->pub_left_hand_joint_control_->publish(joint_trajectory[1]);
    this->pub_right_arm_joint_control_->publish(joint_trajectory[2]);
    this->pub_right_hand_joint_control_->publish(joint_trajectory[3]);
    this->pub_body_joint_control_->publish(joint_trajectory[4]);
    this->pub_head_joint_control_->publish(joint_trajectory[5]);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      builtin_interfaces::msg::Duration dt;
      dt.sec = 0;
      dt.nanosec = static_cast<uint32_t>(0.1 * 10E9);
      std::vector<trajectory_msgs::msg::JointTrajectory> stop_joints = set_joints({}, {}, dt);
      this->pub_left_arm_joint_control_->publish(stop_joints[0]);
      this->pub_left_hand_joint_control_->publish(stop_joints[1]);
      this->pub_right_arm_joint_control_->publish(stop_joints[2]);
      this->pub_right_hand_joint_control_->publish(stop_joints[3]);
      this->pub_body_joint_control_->publish(stop_joints[4]);
      this->pub_head_joint_control_->publish(stop_joints[5]);
      return;
    }

    auto feedback = std::make_shared<MoveJoint::Feedback>();
    feedback->current_joint_names = goal->target_joint_names;
    for (const auto &joint_name : goal->target_joint_names) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();

  }

  // Check if goal was reached
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[goal->target_joint_names[i]] - goal->target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::exe_move_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  // Check if the pose name is valid
  if (std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams &pose) { return pose.pose_name == goal->pose_name; }) == poses_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pose name: %s", goal->pose_name.c_str());
    result->success = false;
    result->message = "Invalid pose name: " + goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Get the target joint rad from the pose name
  std::vector<double> target_joint_rad;
  for (const auto &pose : poses_) {
    if (pose.pose_name == goal->pose_name) {
      target_joint_rad.push_back(pose.arm_right_shoulder_tilt);
      target_joint_rad.push_back(pose.arm_right_upper_roll);
      target_joint_rad.push_back(pose.arm_right_upper_flex);
      target_joint_rad.push_back(pose.arm_right_elbow);
      target_joint_rad.push_back(pose.arm_right_wrist_tilt);
      target_joint_rad.push_back(pose.arm_right_wrist_roll);
      target_joint_rad.push_back(pose.hand_right_finger_l_mcp);
      target_joint_rad.push_back(pose.hand_right_finger_l_dip);
      target_joint_rad.push_back(pose.hand_right_finger_l_pip);
      target_joint_rad.push_back(pose.hand_right_finger_c_mcp);
      target_joint_rad.push_back(pose.hand_right_finger_c_ip);
      target_joint_rad.push_back(pose.hand_right_finger_r_dip);
      target_joint_rad.push_back(pose.hand_right_finger_r_pip);
      target_joint_rad.push_back(pose.arm_left_shoulder_tilt);
      target_joint_rad.push_back(pose.arm_left_upper_roll);
      target_joint_rad.push_back(pose.arm_left_upper_flex);
      target_joint_rad.push_back(pose.arm_left_elbow);
      target_joint_rad.push_back(pose.arm_left_wrist_tilt);
      target_joint_rad.push_back(pose.arm_left_wrist_roll);
      target_joint_rad.push_back(pose.hand_left_finger_l_mcp);
      target_joint_rad.push_back(pose.hand_left_finger_l_dip);
      target_joint_rad.push_back(pose.hand_left_finger_l_pip);
      target_joint_rad.push_back(pose.hand_left_finger_c_mcp);
      target_joint_rad.push_back(pose.hand_left_finger_c_ip);
      target_joint_rad.push_back(pose.hand_left_finger_r_dip);
      target_joint_rad.push_back(pose.hand_left_finger_r_pip);
      target_joint_rad.push_back(pose.body_lift);
      target_joint_rad.push_back(pose.head_pan);
      target_joint_rad.push_back(pose.head_tilt);
      break;
    }
  }

  if (target_joint_rad.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to not find the pose name : %s", goal->pose_name.c_str());

    result->success = false;
    result->message = "[FAIL] Failed to not find the pose name : " +  goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
  }

  // Publish the joint trajectory
  std::vector<trajectory_msgs::msg::JointTrajectory> joint_trajectory;
  joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance);

  try {
    this->pub_left_arm_joint_control_->publish(joint_trajectory[0]);
    this->pub_left_hand_joint_control_->publish(joint_trajectory[1]);
    this->pub_right_arm_joint_control_->publish(joint_trajectory[2]);
    this->pub_right_hand_joint_control_->publish(joint_trajectory[3]);
    this->pub_body_joint_control_->publish(joint_trajectory[4]);
    this->pub_head_joint_control_->publish(joint_trajectory[5]);
    } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      builtin_interfaces::msg::Duration dt;
      dt.sec = 0;
      dt.nanosec = static_cast<uint32_t>(0.1 * 10E9);
      std::vector<trajectory_msgs::msg::JointTrajectory> stop_joints = set_joints({}, {}, dt);
      this->pub_left_arm_joint_control_->publish(stop_joints[0]);
      this->pub_left_hand_joint_control_->publish(stop_joints[1]);
      this->pub_right_arm_joint_control_->publish(stop_joints[2]);
      this->pub_right_hand_joint_control_->publish(stop_joints[3]);
      this->pub_body_joint_control_->publish(stop_joints[4]);
      this->pub_head_joint_control_->publish(stop_joints[5]);
      return;
    }

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->current_joint_names = JointNames;
    for (const auto &joint_name : JointNames) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < JointNames.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[JointNames[i]] - target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->message = "[SUCCESS] Goal has been succeeded";
  result->success = true;
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::serve_move_hand_to_coord(
  const std::shared_ptr<MoveHandToTargetCoord::Request> request,
  std::shared_ptr<MoveHandToTargetCoord::Response> response,
  bool is_right, bool is_one_rink)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->target_coord.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";

  // Transform to robot base from 'sobit_home/base_footprint'
  try{
    goal_coord = tf_buffer_->transform(
      request->target_coord, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // SOBIT HOMEならではの例外．物体が近すぎるので回転じゃどうにもならない場合．
  double lidar_distance_2 = 0.0302345;
  double r = std::sqrt(std::pow(lidar_distance_2, 2));
  if ((std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2)) < 0) {

    response->success = false;
    response->message = "[FAIL] The coordinates are too close to the robot.";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // target_yawにロボットの回転角度を代入
  // calculate the target_yaw to move base of grasping object 
  double target_linear, target_yaw;
  double shoulder_rotate_x, shoulder_rotate_y;
  if (is_right) {
    shoulder_rotate_x = (std::pow(r, 2)*goal_coord.transform.translation.x + r*goal_coord.transform.translation.y*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    shoulder_rotate_y = (std::pow(r, 2)*goal_coord.transform.translation.y - r*goal_coord.transform.translation.x*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    target_yaw =  M_PI / 2. + std::atan2(shoulder_rotate_y, shoulder_rotate_x);
  } else {
    shoulder_rotate_x = (std::pow(r, 2)*goal_coord.transform.translation.x - r*goal_coord.transform.translation.y*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    shoulder_rotate_y = (std::pow(r, 2)*goal_coord.transform.translation.y + r*goal_coord.transform.translation.x*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    target_yaw = -M_PI / 2. + std::atan2(shoulder_rotate_y, shoulder_rotate_x);
  }
  // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる


  // Inverse kinematics to get the target joint rad
  // 座標を元に逆運動学でbody_lift_joint, shoulder_tilt_joint, upper_flex_joint, elbow_joint, wrist_tilt_jointの5つのなすべき角度をvectorで算出
  std::vector<std::string> target_joint_names = {"body_lift_joint","_shoulder_tilt_joint", "_upper_flex_joint", "_elbow_joint", "_wrist_tilt_joint"};
  for (size_t i=1; i<target_joint_names.size(); i++) target_joint_names[i] = (is_right) ? ("arm_right" + target_joint_names[i]) : ("arm_left" + target_joint_names[i]);
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, is_right, is_one_rink, target_yaw);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    response->message = "[FAIL] The target position is too low or tall (z: " + std::to_string(-(LengthShoulderElbow + LengthElbowWrist)) + " <= Grasp Able <= " + std::to_string(LengthShoulderElbow + LengthElbowWrist) + ")";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad, is_right, target_yaw);

  if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
    target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  } else {
    target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  }

  response->move_pose.position.x = target_linear;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::serve_move_hand_to_tf(
  const std::shared_ptr<MoveHandToTargetTF::Request> request,
  std::shared_ptr<MoveHandToTargetTF::Response> response,
  bool is_right, bool is_one_rink)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->tf_differential.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";

  geometry_msgs::msg::TransformStamped goal_coord_shift;


  // Transform the target frame based on the differential tf
  try {
    goal_coord_shift = tf_buffer_->lookupTransform(
      request->tf_differential.header.frame_id, request->target_frame,
      tf2::TimePointZero);

    geometry_msgs::msg::Vector3 euler_target, euler_shift;
    euler_target = get_euler_from_quat(goal_coord_shift.transform.rotation);
    euler_shift = get_euler_from_quat(request->tf_differential.transform.rotation);
    euler_target.x += euler_shift.x;
    euler_target.y += euler_shift.y;
    euler_target.z += euler_shift.z;

    goal_coord_shift.transform.translation.x += request->tf_differential.transform.translation.x;
    goal_coord_shift.transform.translation.y += request->tf_differential.transform.translation.y;
    goal_coord_shift.transform.translation.z += request->tf_differential.transform.translation.z;
    goal_coord_shift.transform.rotation = get_quat_from_euler(euler_target);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform: %s to %s: %s", request->target_frame.c_str(), request->tf_differential.header.frame_id.c_str(),ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform: " + request->target_frame + " to: " + request->tf_differential.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // Transform to robot base from 'sobit_home/base_footprint'
  try{
    goal_coord = tf_buffer_->transform(
      goal_coord_shift, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform coords to %s: %s", goal_coord.header.frame_id.c_str(), ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // SOBIT HOMEならではの例外．物体が近すぎるので回転じゃどうにもならない場合．
  double lidar_distance_2 = 0.0302345;
  double r = std::sqrt(std::pow(lidar_distance_2, 2));
  if ((std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2)) < 0) {

    response->success = false;
    response->message = "[FAIL] The coordinates are too close to the robot.";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // target_yawにロボットの回転角度を代入
  // calculate the target_yaw to move base of grasping object 
  double target_linear, target_yaw;
  double shoulder_rotate_x, shoulder_rotate_y;
  if (is_right) {
    shoulder_rotate_x = (std::pow(r, 2)*goal_coord.transform.translation.x + r*goal_coord.transform.translation.y*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    shoulder_rotate_y = (std::pow(r, 2)*goal_coord.transform.translation.y - r*goal_coord.transform.translation.x*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    target_yaw =  M_PI / 2. + std::atan2(shoulder_rotate_y, shoulder_rotate_x);
  } else {
    shoulder_rotate_x = (std::pow(r, 2)*goal_coord.transform.translation.x - r*goal_coord.transform.translation.y*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    shoulder_rotate_y = (std::pow(r, 2)*goal_coord.transform.translation.y + r*goal_coord.transform.translation.x*std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)-std::pow(r,2))) / (std::pow(goal_coord.transform.translation.x,2) + std::pow(goal_coord.transform.translation.y,2));
    target_yaw = -M_PI / 2. + std::atan2(shoulder_rotate_y, shoulder_rotate_x);
  }
  // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる


  // Inverse kinematics to get the target joint rad
  // 座標を元に逆運動学でbody_lift_joint, shoulder_tilt_joint, upper_flex_joint, elbow_joint, wrist_tilt_jointの5つのなすべき角度をvectorで算出
  std::vector<std::string> target_joint_names = {"body_lift_joint","_shoulder_tilt_joint", "_upper_flex_joint", "_elbow_joint", "_wrist_tilt_joint"};
  for (size_t i=1; i<target_joint_names.size(); i++) target_joint_names[i] = (is_right) ? ("arm_right" + target_joint_names[i]) : ("arm_left" + target_joint_names[i]);
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, is_right, is_one_rink, target_yaw);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    response->message = "[FAIL] The target position is too low or tall (z: " + std::to_string(-(LengthShoulderElbow + LengthElbowWrist)) + " <= Grasp Able <= " + std::to_string(LengthShoulderElbow + LengthElbowWrist) + ")";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad, is_right, target_yaw);

  if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
    target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  } else {
    target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  }

  response->move_pose.position.x = target_linear;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received joint state");

  for (size_t i = 0; i < msg->name.size(); i++) {
    // if (msg->name[i] == "SUB JOINT") continue;  // Skip sub joints

    this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }
}

std::vector<trajectory_msgs::msg::JointTrajectory> JointActionServer::set_joints(
  const std::vector<std::string> &target_joint_names,
  const std::vector<double> &target_joint_rad,
  const builtin_interfaces::msg::Duration &time_allowance)
{
  // Get current joint state from kCurrentJointState
  std::vector<double> full_target_joint_rad;
  for (size_t i = 0; i < JointNames.size(); i++) {
    full_target_joint_rad.push_back(this->curt_joint_state_[JointNames[i]]);
  }
  
  // Update the target joint rad
  for (size_t i = 0; i < target_joint_names.size(); i++) {
    auto it = std::find(JointNames.begin(), JointNames.end(), target_joint_names[i]);
    full_target_joint_rad[std::distance(JointNames.begin(), it)] = target_joint_rad[i];
  }

  auto joint_trajectory_arm_left = trajectory_msgs::msg::JointTrajectory();
  auto joint_trajectory_arm_right = trajectory_msgs::msg::JointTrajectory();
  auto joint_trajectory_hand_left = trajectory_msgs::msg::JointTrajectory();
  auto joint_trajectory_hand_right = trajectory_msgs::msg::JointTrajectory();
  auto joint_trajectory_body = trajectory_msgs::msg::JointTrajectory();
  auto joint_trajectory_head = trajectory_msgs::msg::JointTrajectory();

  joint_trajectory_arm_left.header.stamp = this->now();
  joint_trajectory_arm_left.points.resize(1);
  joint_trajectory_arm_left.points[0].time_from_start = time_allowance;
  joint_trajectory_arm_right.header.stamp = this->now();
  joint_trajectory_arm_right.points.resize(1);
  joint_trajectory_arm_right.points[0].time_from_start = time_allowance;
  joint_trajectory_hand_left.header.stamp = this->now();
  joint_trajectory_hand_left.points.resize(1);
  joint_trajectory_hand_left.points[0].time_from_start = time_allowance;
  joint_trajectory_hand_right.header.stamp = this->now();
  joint_trajectory_hand_right.points.resize(1);
  joint_trajectory_hand_right.points[0].time_from_start = time_allowance;
  joint_trajectory_body.header.stamp = this->now();
  joint_trajectory_body.points.resize(1);
  joint_trajectory_body.points[0].time_from_start = time_allowance;
  joint_trajectory_head.header.stamp = this->now();
  joint_trajectory_head.points.resize(1);
  joint_trajectory_head.points[0].time_from_start = time_allowance;

  for (size_t i = 0; i < JointNames.size(); i++) {
    for (size_t j = 0; j < JointNamesArmLeft.size(); j++) {
      if (JointNames[i] == JointNamesArmLeft[j]) {
        joint_trajectory_arm_left.points[0].positions.push_back(full_target_joint_rad[i]);
        joint_trajectory_arm_left.joint_names.push_back(JointNames[i]);
        break;
      }
    }
    for (size_t j = 0; j < JointNamesHandLeft.size(); j++) {
      if (JointNames[i] == JointNamesHandLeft[j]) {
        joint_trajectory_hand_left.points[0].positions.push_back(full_target_joint_rad[i]);
        joint_trajectory_hand_left.joint_names.push_back(JointNames[i]);
        break;
      }
    }
    for (size_t j = 0; j < JointNamesArmRight.size(); j++) {
      if (JointNames[i] == JointNamesArmRight[j]) {
        joint_trajectory_arm_right.points[0].positions.push_back(full_target_joint_rad[i]);
        joint_trajectory_arm_right.joint_names.push_back(JointNames[i]);
        break;
      }
    }
    for (size_t j = 0; j < JointNamesHandRight.size(); j++) {
      if (JointNames[i] == JointNamesHandRight[j]) {
        joint_trajectory_hand_right.points[0].positions.push_back(full_target_joint_rad[i]);
        joint_trajectory_hand_right.joint_names.push_back(JointNames[i]);
        break;
      }
    }
    for (size_t j = 0; j < JointNamesBody.size(); j++) {
      if (JointNames[i] == JointNamesBody[j]) {
        joint_trajectory_body.points[0].positions.push_back(full_target_joint_rad[i]);
        joint_trajectory_body.joint_names.push_back(JointNames[i]);
        break;
      }
    }
    for (size_t j = 0; j < JointNamesHead.size(); j++) {
      if (JointNames[i] == JointNamesHead[j]) {
        joint_trajectory_head.points[0].positions.push_back(full_target_joint_rad[i]);
        joint_trajectory_head.joint_names.push_back(JointNames[i]);
        break;
      }
    }

    // // Add sub joints
    // if (joint_trajectory.joint_names[i] == JointNames[JointIds::SUB_JOINT]) {
    //   joint_trajectory.points[0].positions.push_back(-full_target_joint_rad[i]);
    //   joint_trajectory.joint_names.push_back("SUB JOINT");
    // }
  }

  return {joint_trajectory_arm_left, joint_trajectory_hand_left, joint_trajectory_arm_right, joint_trajectory_hand_right, joint_trajectory_body, joint_trajectory_head};
}

// ここは，もしも今後逆運動学が3次元に発展したときに，それに対応させるために3次元での順運動学を算出
geometry_msgs::msg::TransformStamped JointActionServer::forward_kinematics(
  const std::vector<double> &target_joint_rad,
  const bool is_right,
  const double target_yaw)
{
  geometry_msgs::msg::TransformStamped final_coord;

  // 上下の+/-を統一するために一時的にinverse_kinematics関数の最後に調整された+/-をもう一度戻す
  std::vector<double> set_joint_rad = target_joint_rad;
  if (is_right) {
    set_joint_rad[2] = target_joint_rad[2] * -1;
  } else {
    set_joint_rad[1] = target_joint_rad[1] * -1;
    set_joint_rad[3] = target_joint_rad[3] * -1;
    set_joint_rad[4] = target_joint_rad[4] * -1;
  }

  // hand_pt <=> final_coord
  geometry_msgs::msg::Point shoulder_pt, elbow_pt, wrist_pt, shoulder_dummy_pt, elbow_dummy_pt, wrist_dummy_pt; // dummy_pt is needed for 3D kinematics...

  shoulder_pt.x = BaseToShoulderDX;
  shoulder_pt.y = (is_right) ? (-BaseToShoulderDY) : (BaseToShoulderDY);
  shoulder_pt.z = BaseToShoulderDZ + target_joint_rad[0]; 
  shoulder_dummy_pt.x = shoulder_pt.x + std::cos(set_joint_rad[1]);
  shoulder_dummy_pt.y = shoulder_pt.y;
  shoulder_dummy_pt.z = shoulder_pt.z + std::sin(set_joint_rad[1]);

  elbow_pt.x = shoulder_pt.x - LengthShoulderElbow * std::sin(set_joint_rad[2]) * std::sin(set_joint_rad[1]);
  if (is_right) elbow_pt.y = shoulder_pt.y + LengthShoulderElbow * std::cos(set_joint_rad[2]);
  else          elbow_pt.y = shoulder_pt.y - LengthShoulderElbow * std::cos(set_joint_rad[2]);
  elbow_pt.z = shoulder_pt.z + LengthShoulderElbow * std::sin(set_joint_rad[2]) * std::cos(set_joint_rad[1]);
  elbow_dummy_pt.x = shoulder_dummy_pt.x + elbow_pt.x - shoulder_pt.x;
  elbow_dummy_pt.y = shoulder_dummy_pt.y + elbow_pt.y - shoulder_pt.y;
  elbow_dummy_pt.z = shoulder_dummy_pt.z + elbow_pt.z - shoulder_pt.z;

  wrist_pt.x = elbow_pt.x
                + (LengthElbowWrist / LengthShoulderElbow) * (elbow_pt.x       - shoulder_pt.x) * std::cos(set_joint_rad[3])
                +  LengthElbowWrist                        * (elbow_dummy_pt.x - elbow_pt.x   ) * std::sin(set_joint_rad[3]);
  wrist_pt.y = elbow_pt.y
                + (LengthElbowWrist / LengthShoulderElbow) * (elbow_pt.y       - shoulder_pt.y) * std::cos(set_joint_rad[3])
                +  LengthElbowWrist                        * (elbow_dummy_pt.y - elbow_pt.y   ) * std::sin(set_joint_rad[3]);
  wrist_pt.z = elbow_pt.z
                + (LengthElbowWrist / LengthShoulderElbow) * (elbow_pt.z       - shoulder_pt.z) * std::cos(set_joint_rad[3])
                +  LengthElbowWrist                        * (elbow_dummy_pt.z - elbow_pt.z   ) * std::sin(set_joint_rad[3]);

  // 算出上必要となる各種パラメータの設定．
  double alpha, beta, gamma/*, delta*/; // 算出上必要な平面がありその方程式を[alpha*x + beta*y + gamma*z + delta = 0]としたときの4つのパラメータ
  alpha = (elbow_pt.y - shoulder_pt.y)*(shoulder_dummy_pt.z - shoulder_pt.z) - (elbow_pt.z - shoulder_pt.z)*(shoulder_dummy_pt.y - shoulder_pt.y);
  beta  = (elbow_pt.z - shoulder_pt.z)*(shoulder_dummy_pt.x - shoulder_pt.x) - (elbow_pt.x - shoulder_pt.x)*(shoulder_dummy_pt.z - shoulder_pt.z);
  gamma = (elbow_pt.x - shoulder_pt.x)*(shoulder_dummy_pt.y - shoulder_pt.y) - (elbow_pt.y - shoulder_pt.y)*(shoulder_dummy_pt.x - shoulder_pt.x);
  // delta = -(alpha*shoulder_pt.x + beta*shoulder_pt.y + gamma*shoulder_pt.z);
  geometry_msgs::msg::Vector3 n_hand;
  n_hand.x = beta  * (wrist_pt.z - elbow_pt.z) - gamma * (wrist_pt.y - elbow_pt.y);
  n_hand.y = gamma * (wrist_pt.x - elbow_pt.x) - alpha * (wrist_pt.z - elbow_pt.z);
  n_hand.z = alpha * (wrist_pt.y - elbow_pt.y) - beta  * (wrist_pt.x - elbow_pt.x);

  wrist_dummy_pt.x = wrist_pt.x + (n_hand.x / std::sqrt(std::pow(n_hand.x,2) + std::pow(n_hand.y,2) + std::pow(n_hand.z,2)));
  wrist_dummy_pt.y = wrist_pt.y + (n_hand.y / std::sqrt(std::pow(n_hand.x,2) + std::pow(n_hand.y,2) + std::pow(n_hand.z,2)));
  wrist_dummy_pt.z = wrist_pt.z + (n_hand.z / std::sqrt(std::pow(n_hand.x,2) + std::pow(n_hand.y,2) + std::pow(n_hand.z,2)));

  final_coord.transform.translation.x = wrist_pt.x
                                        + (LengthHand / LengthElbowWrist) * (wrist_pt.x       - elbow_pt.x) * std::cos(set_joint_rad[4])
                                        +  LengthHand                     * (wrist_dummy_pt.x - wrist_pt.x) * std::sin(set_joint_rad[4]);
  final_coord.transform.translation.y = wrist_pt.y
                                        + (LengthHand / LengthElbowWrist) * (wrist_pt.y       - elbow_pt.y) * std::cos(set_joint_rad[4])
                                        +  LengthHand                     * (wrist_dummy_pt.y - wrist_pt.y) * std::sin(set_joint_rad[4]);
  final_coord.transform.translation.z = wrist_pt.z
                                        + (LengthHand / LengthElbowWrist) * (wrist_pt.z       - elbow_pt.z) * std::cos(set_joint_rad[4])
                                        +  LengthHand                     * (wrist_dummy_pt.z - wrist_pt.z) * std::sin(set_joint_rad[4]);

  // TODO Poseの算出::wrist-handのベクトルを姿勢にすればいいのではと予想
  final_coord.transform.rotation.w = 1.;
  final_coord.transform.rotation.x = 0.;
  final_coord.transform.rotation.y = 0.;
  final_coord.transform.rotation.z = 0.;

  // 最後にtarget_yawの回転分を考慮
  double temp_x, temp_y;
  temp_x = final_coord.transform.translation.x;
  temp_y = final_coord.transform.translation.y;
  final_coord.transform.translation.x = temp_x*std::cos(target_yaw) - temp_y*std::sin(target_yaw);
  final_coord.transform.translation.y = temp_y*std::cos(target_yaw) + temp_x*std::sin(target_yaw);

  return final_coord;
}

std::vector<double> JointActionServer::inverse_kinematics(
  const geometry_msgs::msg::TransformStamped &goal_coord,  // 'goal_coord' is the coordinates of robot base.
  const bool is_right, bool is_one_rink,
  const double target_yaw)
{

  // "body_lift","_arm_shoulder_roll_joint", "_arm_shoulder_pan_joint", "_arm_elbow_tilt_joint", "_arm_wrist_tilt_joint"
  // return msg
  std::vector<double> target_joint_rad = {-0.109, 0.0, -M_PI/2., 0.0, 0.0};

  // do 'Hand-Calculate' to a coordinate system valid for this robot's inverse kinematics
  // In case SOBIT HOME, the coordinates are based on the shoulder to be grasped.

  // target_yaw分，回転したあとの座標をgoal_coord_rotateに代入
  geometry_msgs::msg::TransformStamped goal_coord_rotate;
  // ロボット回転後の座標へ変換
  goal_coord_rotate.transform.translation.x = goal_coord.transform.translation.x*std::cos(-target_yaw) - goal_coord.transform.translation.y*std::sin(-target_yaw);
  goal_coord_rotate.transform.translation.y = goal_coord.transform.translation.y*std::cos(-target_yaw) + goal_coord.transform.translation.x*std::sin(-target_yaw);
  goal_coord_rotate.transform.translation.z = goal_coord.transform.translation.z;
  goal_coord_rotate.transform.rotation = goal_coord.transform.rotation;

  // 肩の座標系に変換（SOBIT HOMEはそのようにして逆運動学をする）
  goal_coord_rotate.transform.translation.x -=  BaseToShoulderDX;
  goal_coord_rotate.transform.translation.y -= (is_right ? -BaseToShoulderDY : BaseToShoulderDY);
  goal_coord_rotate.transform.translation.z -=  BaseToShoulderDZ + target_joint_rad[0];

  if (is_one_rink) { // one link graspable mode...
    if ((LengthShoulderElbow + LengthElbowWrist + LengthHand/2.) < std::fabs(goal_coord_rotate.transform.translation.z)) {
      RCLCPP_WARN(this->get_logger(), "The target position is too low or tall (min:%.2f[m] < max:%.2f[m] not in target:%.2f)", -(LengthShoulderElbow + LengthElbowWrist + LengthHand/2.), (LengthShoulderElbow + LengthElbowWrist + LengthHand/2.), goal_coord_rotate.transform.translation.z);
      target_joint_rad.clear();
      return target_joint_rad;
    }
    target_joint_rad[1] = M_PI - std::acos(goal_coord_rotate.transform.translation.z / (LengthShoulderElbow + LengthElbowWrist + LengthHand/2.));
  } else {           // multi link (default) mode...
    if (goal_coord_rotate.transform.translation.z < -(LengthShoulderElbow + LengthElbowWrist) ) {
      RCLCPP_WARN(this->get_logger(), "The target position is too low (%.2f[m] < min:%.2f[m])", goal_coord_rotate.transform.translation.z, -(LengthShoulderElbow + LengthElbowWrist));
      target_joint_rad.clear();
      return target_joint_rad;
    }
    else if ((LengthShoulderElbow + LengthElbowWrist) < goal_coord_rotate.transform.translation.z) {
      RCLCPP_WARN(this->get_logger(), "The target position is too tall (max:%.2f[m] < %.2f[m])", LengthShoulderElbow + LengthElbowWrist, goal_coord_rotate.transform.translation.z);
      target_joint_rad.clear();
      return target_joint_rad;
    }

    // ほぼ肩くらいの高さならば・・・(肩より下)
    if ((-LengthShoulderElbow*std::cos(M_PI/4.) <= goal_coord_rotate.transform.translation.z) && (goal_coord_rotate.transform.translation.z < 0.)) {
      target_joint_rad[1] = M_PI / 4.;
    }
    // ほぼ肘くらいの高さならば・・・(肩より下で肘より上)
    else if ((-(LengthShoulderElbow + LengthElbowWrist*std::cos(M_PI/4.)) <= goal_coord_rotate.transform.translation.z) && (goal_coord_rotate.transform.translation.z < -LengthShoulderElbow*std::cos(M_PI/4.))) {
      target_joint_rad[1] = 0.;
    }
    // ほぼ肩くらいの高さならば・・・(肩より上)
    else if ((0 <= goal_coord_rotate.transform.translation.z) && (goal_coord_rotate.transform.translation.z < LengthElbowWrist*std::cos(M_PI/4.))) {
      target_joint_rad[1] = M_PI / 2.;
    }
    // どれにも該当しないならば・・・
    else {
      target_joint_rad[1] = M_PI - std::atan2(std::sqrt(std::pow(LengthShoulderElbow + LengthElbowWrist, 2) - std::pow(goal_coord_rotate.transform.translation.z, 2)), goal_coord_rotate.transform.translation.z);
    }

    double e_x =  LengthShoulderElbow*std::sin(target_joint_rad[1]);
    double e_z = -LengthShoulderElbow*std::cos(target_joint_rad[1]);
    double w_x =  std::sqrt(std::pow(LengthElbowWrist, 2) - std::pow(goal_coord_rotate.transform.translation.z - e_z, 2)) + e_x;
    double w_z =  goal_coord_rotate.transform.translation.z;

    target_joint_rad[3] = M_PI - std::atan2(w_x - e_x, w_z - e_z) - target_joint_rad[1];
    target_joint_rad[4] = M_PI/2. - (target_joint_rad[1] + target_joint_rad[3]);
  }

  // 右と左で+/-の違いがあるので調整．
  if (is_right) {
    target_joint_rad[2] *= -1;
  } else {
    target_joint_rad[1] *= -1;
    target_joint_rad[3] *= -1;
    target_joint_rad[4] *= -1;
  }

  return target_joint_rad;
}

} // namespace sobit_home
