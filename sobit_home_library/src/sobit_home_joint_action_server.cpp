#include "sobit_home_library/sobit_home_joint_action_server.hpp"

namespace sobit_home
{
  JointActionServer::JointActionServer(const rclcpp::NodeOptions &options)
      : Node("joint_action_server", options),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
        kinematics_(std::make_unique<SobitHomeKinematics>()),
        urdf_loaded_(false)
  {
    rclcpp::QoS qos_profile(1);

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

    this->action_server_move_joints_ = rclcpp_action::create_server<MoveJoint>(
        this, "move_joint",
        std::bind(&JointActionServer::handle_move_joints_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointActionServer::handle_move_joints_cancel, this, std::placeholders::_1),
        std::bind(&JointActionServer::handle_move_joints_accepted, this, std::placeholders::_1));

    this->action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
        this, "move_to_pose",
        std::bind(&JointActionServer::handle_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointActionServer::handle_move_to_pose_cancel, this, std::placeholders::_1),
        std::bind(&JointActionServer::handle_move_to_pose_accepted, this, std::placeholders::_1));

    this->service_get_hand_to_coord_left_ = this->create_service<GetHandToTargetCoord>(
        "get_hand_to_coord/left", [this](const std::shared_ptr<GetHandToTargetCoord::Request> req, std::shared_ptr<GetHandToTargetCoord::Response> res)
        { get_pos_to_coord(req, res, false, false); });
    this->service_get_hand_to_tf_left_ = this->create_service<GetHandToTargetTF>(
        "get_hand_to_tf/left", [this](const std::shared_ptr<GetHandToTargetTF::Request> req, std::shared_ptr<GetHandToTargetTF::Response> res)
        { get_pos_to_tf(req, res, false, false); });
    this->service_get_hand_to_coord_right_ = this->create_service<GetHandToTargetCoord>(
        "get_hand_to_coord/right", [this](const std::shared_ptr<GetHandToTargetCoord::Request> req, std::shared_ptr<GetHandToTargetCoord::Response> res)
        { get_pos_to_coord(req, res, true, false); });
    this->service_get_hand_to_tf_right_ = this->create_service<GetHandToTargetTF>(
        "get_hand_to_tf/right", [this](const std::shared_ptr<GetHandToTargetTF::Request> req, std::shared_ptr<GetHandToTargetTF::Response> res)
        { get_pos_to_tf(req, res, true, false); });

    this->service_get_hand_to_coord_one_left_ = this->create_service<GetHandToTargetCoord>(
        "get_hand_to_coord/one_link/left", [this](const std::shared_ptr<GetHandToTargetCoord::Request> req, std::shared_ptr<GetHandToTargetCoord::Response> res)
        { get_pos_to_coord(req, res, false, true); });
    this->service_get_hand_to_tf_one_left_ = this->create_service<GetHandToTargetTF>(
        "get_hand_to_tf/one_link/left", [this](const std::shared_ptr<GetHandToTargetTF::Request> req, std::shared_ptr<GetHandToTargetTF::Response> res)
        { get_pos_to_tf(req, res, false, true); });
    this->service_get_hand_to_coord_one_right_ = this->create_service<GetHandToTargetCoord>(
        "get_hand_to_coord/one_link/right", [this](const std::shared_ptr<GetHandToTargetCoord::Request> req, std::shared_ptr<GetHandToTargetCoord::Response> res)
        { get_pos_to_coord(req, res, true, true); });
    this->service_get_hand_to_tf_one_right_ = this->create_service<GetHandToTargetTF>(
        "get_hand_to_tf/one_link/right", [this](const std::shared_ptr<GetHandToTargetTF::Request> req, std::shared_ptr<GetHandToTargetTF::Response> res)
        { get_pos_to_tf(req, res, true, true); });

    this->service_get_head_to_coord_ = this->create_service<GetHandToTargetCoord>(
        "get_head_to_coord", [this](const std::shared_ptr<GetHandToTargetCoord::Request> req, std::shared_ptr<GetHandToTargetCoord::Response> res)
        { get_head_to_coord(req, res); });
    this->service_get_head_to_tf_ = this->create_service<GetHandToTargetTF>(
        "get_head_to_tf", [this](const std::shared_ptr<GetHandToTargetTF::Request> req, std::shared_ptr<GetHandToTargetTF::Response> res)
        { get_head_to_tf(req, res); });

    this->service_server_get_finger_angle_ = this->create_service<GetFingerAngle>(
        "get_finger_angle", [this](const std::shared_ptr<GetFingerAngle::Request> req, std::shared_ptr<GetFingerAngle::Response> res)
        { serve_get_finger_angle(req, res); });

    this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", qos_profile, std::bind(&JointActionServer::joint_state_callback, this, std::placeholders::_1));
    this->pub_left_arm_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("arm_left_position_controller/joint_trajectory", qos_profile);
    this->pub_left_hand_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("hand_left_position_controller/joint_trajectory", qos_profile);
    this->pub_right_arm_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("arm_right_position_controller/joint_trajectory", qos_profile);
    this->pub_right_hand_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("hand_right_position_controller/joint_trajectory", qos_profile);
    this->pub_body_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("body_position_controller/joint_trajectory", qos_profile);
    this->pub_head_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("head_position_controller/joint_trajectory", qos_profile);

    this->declare_parameter("robot_description", "");
    this->urdf_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&JointActionServer::load_joint_limits, this));

    this->declare_parameter("poses", std::vector<std::string>());
    auto pose_names = this->get_parameter("poses").as_string_array();

    for (const auto &pose_name : pose_names)
    {
      this->declare_parameter(pose_name + ".arm_right_shoulder_tilt", 0.0);
      this->declare_parameter(pose_name + ".arm_right_upper_roll", 0.0);
      this->declare_parameter(pose_name + ".arm_right_upper_flex", 0.0);
      this->declare_parameter(pose_name + ".arm_right_elbow", 0.0);
      this->declare_parameter(pose_name + ".arm_right_wrist_tilt", 0.0);
      this->declare_parameter(pose_name + ".arm_right_wrist_roll", 0.0);

      this->declare_parameter(pose_name + ".hand_right_finger_l_mcp", 0.0);
      this->declare_parameter(pose_name + ".hand_right_finger_l_pip", 0.0);
      this->declare_parameter(pose_name + ".hand_right_finger_l_dip", 0.0);
      this->declare_parameter(pose_name + ".hand_right_finger_c_mcp", 0.0);
      this->declare_parameter(pose_name + ".hand_right_finger_c_ip", 0.0);
      this->declare_parameter(pose_name + ".hand_right_finger_r_pip", 0.0);
      this->declare_parameter(pose_name + ".hand_right_finger_r_dip", 0.0);

      this->declare_parameter(pose_name + ".arm_left_shoulder_tilt", 0.0);
      this->declare_parameter(pose_name + ".arm_left_upper_roll", 0.0);
      this->declare_parameter(pose_name + ".arm_left_upper_flex", 0.0);
      this->declare_parameter(pose_name + ".arm_left_elbow", 0.0);
      this->declare_parameter(pose_name + ".arm_left_wrist_tilt", 0.0);
      this->declare_parameter(pose_name + ".arm_left_wrist_roll", 0.0);

      this->declare_parameter(pose_name + ".hand_left_finger_l_mcp", 0.0);
      this->declare_parameter(pose_name + ".hand_left_finger_l_pip", 0.0);
      this->declare_parameter(pose_name + ".hand_left_finger_l_dip", 0.0);
      this->declare_parameter(pose_name + ".hand_left_finger_c_mcp", 0.0);
      this->declare_parameter(pose_name + ".hand_left_finger_c_ip", 0.0);
      this->declare_parameter(pose_name + ".hand_left_finger_r_pip", 0.0);
      this->declare_parameter(pose_name + ".hand_left_finger_r_dip", 0.0);

      this->declare_parameter(pose_name + ".body_lift", 0.0);
      this->declare_parameter(pose_name + ".head_pan", 0.0);
      this->declare_parameter(pose_name + ".head_tilt", 0.0);

      PoseParams p;
      p.pose_name = pose_name;

      p.arm_right_shoulder_tilt = this->get_parameter(pose_name + ".arm_right_shoulder_tilt").as_double();
      p.arm_right_upper_roll = this->get_parameter(pose_name + ".arm_right_upper_roll").as_double();
      p.arm_right_upper_flex = this->get_parameter(pose_name + ".arm_right_upper_flex").as_double();
      p.arm_right_elbow = this->get_parameter(pose_name + ".arm_right_elbow").as_double();
      p.arm_right_wrist_tilt = this->get_parameter(pose_name + ".arm_right_wrist_tilt").as_double();
      p.arm_right_wrist_roll = this->get_parameter(pose_name + ".arm_right_wrist_roll").as_double();

      p.hand_right_finger_l_mcp = this->get_parameter(pose_name + ".hand_right_finger_l_mcp").as_double();
      p.hand_right_finger_l_pip = this->get_parameter(pose_name + ".hand_right_finger_l_pip").as_double();
      p.hand_right_finger_l_dip = this->get_parameter(pose_name + ".hand_right_finger_l_dip").as_double();
      p.hand_right_finger_c_mcp = this->get_parameter(pose_name + ".hand_right_finger_c_mcp").as_double();
      p.hand_right_finger_c_ip = this->get_parameter(pose_name + ".hand_right_finger_c_ip").as_double();
      p.hand_right_finger_r_pip = this->get_parameter(pose_name + ".hand_right_finger_r_pip").as_double();
      p.hand_right_finger_r_dip = this->get_parameter(pose_name + ".hand_right_finger_r_dip").as_double();

      p.arm_left_shoulder_tilt = this->get_parameter(pose_name + ".arm_left_shoulder_tilt").as_double();
      p.arm_left_upper_roll = this->get_parameter(pose_name + ".arm_left_upper_roll").as_double();
      p.arm_left_upper_flex = this->get_parameter(pose_name + ".arm_left_upper_flex").as_double();
      p.arm_left_elbow = this->get_parameter(pose_name + ".arm_left_elbow").as_double();
      p.arm_left_wrist_tilt = this->get_parameter(pose_name + ".arm_left_wrist_tilt").as_double();
      p.arm_left_wrist_roll = this->get_parameter(pose_name + ".arm_left_wrist_roll").as_double();

      p.hand_left_finger_l_mcp = this->get_parameter(pose_name + ".hand_left_finger_l_mcp").as_double();
      p.hand_left_finger_l_pip = this->get_parameter(pose_name + ".hand_left_finger_l_pip").as_double();
      p.hand_left_finger_l_dip = this->get_parameter(pose_name + ".hand_left_finger_l_dip").as_double();
      p.hand_left_finger_c_mcp = this->get_parameter(pose_name + ".hand_left_finger_c_mcp").as_double();
      p.hand_left_finger_c_ip = this->get_parameter(pose_name + ".hand_left_finger_c_ip").as_double();
      p.hand_left_finger_r_pip = this->get_parameter(pose_name + ".hand_left_finger_r_pip").as_double();
      p.hand_left_finger_r_dip = this->get_parameter(pose_name + ".hand_left_finger_r_dip").as_double();

      p.body_lift = this->get_parameter(pose_name + ".body_lift").as_double();
      p.head_pan = this->get_parameter(pose_name + ".head_pan").as_double();
      p.head_tilt = this->get_parameter(pose_name + ".head_tilt").as_double();

      poses_.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(), "JointActionServer has been initialized.");
  }

  JointActionServer::~JointActionServer() {}

  rclcpp_action::GoalResponse JointActionServer::handle_move_joints_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoint::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
  rclcpp_action::GoalResponse JointActionServer::handle_move_to_pose_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
  rclcpp_action::CancelResponse JointActionServer::handle_move_joints_cancel(const std::shared_ptr<GoalHandleMoveJoints>) { return rclcpp_action::CancelResponse::ACCEPT; }
  rclcpp_action::CancelResponse JointActionServer::handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose>) { return rclcpp_action::CancelResponse::ACCEPT; }

  void JointActionServer::handle_move_joints_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Accepted move_joints goal request");
    std::thread{std::bind(&JointActionServer::exe_move_joints, this, std::placeholders::_1), goal_handle}.detach();
  }

  void JointActionServer::handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Accepted move_to_pose goal request");
    std::thread{std::bind(&JointActionServer::exe_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
  }

  void JointActionServer::load_joint_limits()
  {
    if (urdf_loaded_)
      return;

    if (!async_param_client_->service_is_ready())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Parameter service not ready for %s", robot_description_source_node_.c_str());
      return;
    }

    if (!robot_desc_requested_)
    {
      robot_desc_future_ = async_param_client_->get_parameters({"robot_description"});
      robot_desc_requested_ = true;
      return;
    }

    if (robot_desc_future_.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready)
    {
      return;
    }

    std::vector<rclcpp::Parameter> params;
    try
    {
      params = robot_desc_future_.get();
    }
    catch (...)
    {
      robot_desc_requested_ = false;
      return;
    }
    robot_desc_requested_ = false;

    if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
      return;

    const std::string urdf_xml = params[0].as_string();
    if (urdf_xml.empty())
      return;

    if (!parse_urdf_limits(urdf_xml))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to parse URDF limits");
      return;
    }

    urdf_loaded_ = true;
    urdf_timer_.reset();
    RCLCPP_INFO(get_logger(), "Joint limits loaded (%zu joints)", joint_limits_.size());
  }

  bool JointActionServer::parse_urdf_limits(const std::string &urdf_xml)
  {
    urdf::Model model;

    if (!model.initString(urdf_xml))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
      return false;
    }

    joint_limits_.clear();

    for (const auto &joint_pair : model.joints_)
    {

      const auto &joint = joint_pair.second;

      if (!joint)
        continue;

      Limit lim;
      lim.has = false;

      if (joint->limits)
      {
        lim.lower = joint->limits->lower;
        lim.upper = joint->limits->upper;
        lim.velocity = joint->limits->velocity;
        lim.effort = joint->limits->effort;
        lim.has = true;

        joint_limits_[joint->name] = lim;
      }
    }

    return true;
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

  void JointActionServer::exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveJoint::Result>();

    if (goal->target_joint_names.size() != goal->target_joint_rad.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Joint names and rad size mismatch.");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    auto publish_group = [&](auto &pub, const std::string &grp)
    {
      auto traj = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, grp);
      if (!traj.joint_names.empty())
      {
        try
        {
          pub->publish(traj);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Publish failed in %s: %s", grp.c_str(), e.what());
        }
      }
    };

    publish_group(pub_left_arm_joint_control_, "arm_left");
    publish_group(pub_right_arm_joint_control_, "arm_right");
    publish_group(pub_left_hand_joint_control_, "hand_left");
    publish_group(pub_right_hand_joint_control_, "hand_right");
    publish_group(pub_body_joint_control_, "body");
    publish_group(pub_head_joint_control_, "head");

    auto start_time = this->now();
    while (this->now() - start_time < goal->time_allowance)
    {
      if (goal_handle->is_canceling())
      {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance)
  {
    if (goal_handle->is_canceling())
    {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      return;
    }

    auto feedback = std::make_shared<MoveJoint::Feedback>();
    feedback->current_joint_names = goal->target_joint_names;
    for (const auto &joint_name : goal->target_joint_names)
    {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < goal->target_joint_names.size(); i++)
  {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[goal->target_joint_names[i]] - goal->target_joint_rad[i]) > 0.1)
    {
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
  if (std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams &pose)
                   { return pose.pose_name == goal->pose_name; }) == poses_.end())
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid pose name: %s", goal->pose_name.c_str());
    result->success = false;
    result->message = "Invalid pose name: " + goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Get the target joint rad from the pose name
  std::vector<double> target_joint_rad(JointIds::JointNum);
  for (const auto &pose : poses_)
  {
    if (pose.pose_name == goal->pose_name)
    {
      target_joint_rad[JointIds::Arm_R_Shoulder_Tilt_Joint] = pose.arm_right_shoulder_tilt;
      target_joint_rad[JointIds::Arm_R_Upper_Roll_Joint] = pose.arm_right_upper_roll;
      target_joint_rad[JointIds::Arm_R_Upper_Flex_Joint] = pose.arm_right_upper_flex;
      target_joint_rad[JointIds::Arm_R_Elbow_Joint] = pose.arm_right_elbow;
      target_joint_rad[JointIds::Arm_R_Wrist_Tilt_Joint] = pose.arm_right_wrist_tilt;
      target_joint_rad[JointIds::Arm_R_Wrist_Roll_Joint] = pose.arm_right_wrist_roll;

      // target_joint_rad[JointIds::Hand_R_Finger_L_MCP] = pose.hand_right_finger_l_mcp;
      // target_joint_rad[JointIds::Hand_R_Finger_L_DIP] = pose.hand_right_finger_l_dip;
      // target_joint_rad[JointIds::Hand_R_Finger_L_PIP] = pose.hand_right_finger_l_pip;
      // target_joint_rad[JointIds::Hand_R_Finger_C_MCP] = pose.hand_right_finger_c_mcp;
      // target_joint_rad[JointIds::Hand_R_Finger_C_IP] = pose.hand_right_finger_c_ip;
      // target_joint_rad[JointIds::Hand_R_Finger_R_PIP] = pose.hand_right_finger_r_pip;
      // target_joint_rad[JointIds::Hand_R_Finger_R_DIP] = pose.hand_right_finger_r_dip;

      target_joint_rad[JointIds::Arm_L_Shoulder_Tilt_Joint] = pose.arm_left_shoulder_tilt;
      target_joint_rad[JointIds::Arm_L_Upper_Roll_Joint] = pose.arm_left_upper_roll;
      target_joint_rad[JointIds::Arm_L_Upper_Flex_Joint] = pose.arm_left_upper_flex;
      target_joint_rad[JointIds::Arm_L_Elbow_Joint] = pose.arm_left_elbow;
      target_joint_rad[JointIds::Arm_L_Wrist_Tilt_Joint] = pose.arm_left_wrist_tilt;
      target_joint_rad[JointIds::Arm_L_Wrist_Roll_Joint] = pose.arm_left_wrist_roll;

      // target_joint_rad[JointIds::Hand_L_Finger_L_MCP] = pose.hand_left_finger_l_mcp;
      // target_joint_rad[JointIds::Hand_L_Finger_L_DIP] = pose.hand_left_finger_l_dip;
      // target_joint_rad[JointIds::Hand_L_Finger_L_PIP] = pose.hand_left_finger_l_pip;
      // target_joint_rad[JointIds::Hand_L_Finger_C_MCP] = pose.hand_left_finger_c_mcp;
      // target_joint_rad[Jooints::Hand_L_Finger_C_IP] = pose.hand_left_finger_c_ip;
      // target_joint_rad[JointIds::Hand_L_Finger_R_DIP] = pose.hand_left_finger_r_pip;
      // target_joint_rad[JointIds::Hand_L_Finger_R_PIP] = pose.hand_left_finger_r_dip;

      target_joint_rad[JointIds::Body_Lift_Joint] = pose.body_lift;
      target_joint_rad[JointIds::Head_Pan_Joint] = pose.head_pan;
      target_joint_rad[JointIds::Head_Tilt_Joint] = pose.head_tilt;
      break;
    }
  }

  if (target_joint_rad.size() == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to not find the pose name : %s", goal->pose_name.c_str());

    result->success = false;
    result->message = "[FAIL] Failed to not find the pose name : " + goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
  }

  // Create joint list excluding hands
  std::vector<std::string> joints_without_hands;
  for (const auto &joint_name : JointNames)
  {
    if (std::find(JointNamesHandLeft.begin(), JointNamesHandLeft.end(), joint_name) == JointNamesHandLeft.end() &&
        std::find(JointNamesHandRight.begin(), JointNamesHandRight.end(), joint_name) == JointNamesHandRight.end())
    {
      joints_without_hands.push_back(joint_name);
    }
  }

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory arm_left_joint_trajectory;
  // trajectory_msgs::msg::JointTrajectory hand_left_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory arm_right_joint_trajectory;
  // trajectory_msgs::msg::JointTrajectory hand_right_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory body_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory head_joint_trajectory;
  arm_left_joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance, "arm_left");
  arm_right_joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance, "arm_right");
  // hand_left_joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance, "hand_left");
  // hand_right_joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance, "hand_right");
  body_joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance, "body");
  head_joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance, "head");
  try
  {
    if (!arm_left_joint_trajectory.joint_names.empty())
      this->pub_left_arm_joint_control_->publish(arm_left_joint_trajectory);
    if (!arm_right_joint_trajectory.joint_names.empty())
      this->pub_right_arm_joint_control_->publish(arm_right_joint_trajectory);
    // if (!hand_left_joint_trajectory.joint_names.empty())
    //   this->pub_left_hand_joint_control_->publish(hand_left_joint_trajectory);
    // if (!hand_right_joint_trajectory.joint_names.empty())
    //   this->pub_right_hand_joint_control_->publish(hand_right_joint_trajectory);
    if (!body_joint_trajectory.joint_names.empty())
      this->pub_body_joint_control_->publish(body_joint_trajectory);
    if (!head_joint_trajectory.joint_names.empty())
      this->pub_head_joint_control_->publish(head_joint_trajectory);
  }
  catch (const std::exception &ex)
  {
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

  while (this->now() - start_time < goal->time_allowance)
  {
    if (goal_handle->is_canceling())
    {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      return;
    }

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->current_joint_names = JointNames;
    for (const auto &joint_name : JointNames)
    {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < JointNames.size(); i++)
  {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[JointNames[i]] - target_joint_rad[i]) > 0.1)
    {
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

void JointActionServer::get_pos_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request, std::shared_ptr<GetHandToTargetCoord::Response> response, bool is_right, bool is_one_rink)
{
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";
  try
  {
    goal_coord = tf_buffer_->transform(request->target_coord, goal_coord.header.frame_id, tf2::durationFromSec(1.0));
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
    response->success = false;
    return;
  }

  double target_yaw = (is_right ? 1.0 : -1.0) * M_PI / 2.0 + std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x);
  auto rads = kinematics_->inverse_kinematics(goal_coord, is_right, is_one_rink, target_yaw);

  if (rads.empty())
  {
    response->success = false;
    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = kinematics_->forward_kinematics(rads, is_right, target_yaw);
  double dist = std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x, 2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y, 2));

  response->move_pose.position.x = (std::sqrt(std::pow(hand_pose.transform.translation.x, 2) + std::pow(hand_pose.transform.translation.y, 2)) < std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y, 2))) ? dist : -dist;
  geometry_msgs::msg::Vector3 euler;
  euler.z = target_yaw;
  response->move_pose.orientation = kinematics_->get_quat_from_euler(euler);
  response->target_joint_rad = rads;
  response->target_joint_names = {"body_lift_joint"};
  std::string side = is_right ? "arm_right" : "arm_left";
  response->target_joint_names.insert(response->target_joint_names.end(), {side + "_shoulder_tilt_joint", side + "_upper_flex_joint", side + "_elbow_joint", side + "_wrist_tilt_joint"});
  response->success = true;
}

void JointActionServer::exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoint::Result>();

  if (goal->target_joint_names.size() != goal->target_joint_rad.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Joint names and rad size mismatch.");
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  auto publish_group = [&](auto &pub, const std::string &grp)
  {
    auto traj = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, grp);
    if (!traj.joint_names.empty())
    {
      try
      {
        pub->publish(traj);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Publish failed in %s: %s", grp.c_str(), e.what());
      }
    }
  };

  publish_group(pub_left_arm_joint_control_, "arm_left");
  publish_group(pub_right_arm_joint_control_, "arm_right");
  publish_group(pub_left_hand_joint_control_, "hand_left");
  publish_group(pub_right_hand_joint_control_, "hand_right");
  publish_group(pub_body_joint_control_, "body");
  publish_group(pub_head_joint_control_, "head");

  auto start_time = this->now();
  while (this->now() - start_time < goal->time_allowance)
  {
    if (goal_handle->is_canceling())
    {
      result->success = false;
      goal_handle->canceled(result);
      return;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  result->success = true;
  goal_handle->succeed(result);
}

void JointActionServer::get_pos_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request, std::shared_ptr<GetHandToTargetCoord::Response> response, bool is_right, bool is_one_rink)
{
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";
  try
  {
    goal_coord = tf_buffer_->transform(request->target_coord, goal_coord.header.frame_id, tf2::durationFromSec(1.0));
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
    response->success = false;
    return;
  }

  double target_yaw = (is_right ? 1.0 : -1.0) * M_PI / 2.0 + std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x);
  auto rads = kinematics_->inverse_kinematics(goal_coord, is_right, is_one_rink, target_yaw);

  if (rads.empty())
  {
    response->success = false;
    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad, is_right, target_yaw);

  if (std::sqrt(std::pow(hand_pose.transform.translation.x, 2) + std::pow(hand_pose.transform.translation.y, 2)) < std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y, 2)))
  {
    target_linear = std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x, 2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y, 2));
  }
  else
  {
    target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x, 2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y, 2));
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

void JointActionServer::serve_get_finger_angle(
    const std::shared_ptr<GetFingerAngle::Request> request,
    std::shared_ptr<GetFingerAngle::Response> response)
{
  std::vector<std::string> target_joint_names;
  std::vector<double> opened_target_joint_rad;
  std::vector<double> closed_target_joint_rad;
  std::vector<bool> invert_target_joint;

  double sign_multiplier = 1.0;

  invert_target_joint = {true, false, false, true, true, false, false};

  if (request->is_right)
  {
    target_joint_names = {"hand_right_finger_l_mcp_joint", "hand_right_finger_l_pip_joint", "hand_right_finger_l_dip_joint",
                          "hand_right_finger_c_mcp_joint", "hand_right_finger_c_ip_joint", "hand_right_finger_r_pip_joint",
                          "hand_right_finger_r_dip_joint"};
    sign_multiplier = -1.0;
  }
  else
  {
    target_joint_names = {"hand_left_finger_l_mcp_joint", "hand_left_finger_l_pip_joint", "hand_left_finger_l_dip_joint",
                          "hand_left_finger_c_mcp_joint", "hand_left_finger_c_ip_joint", "hand_left_finger_r_pip_joint",
                          "hand_left_finger_r_dip_joint"};
  }

  if (request->grasp_form == 0)
  {
    opened_target_joint_rad = {-1.65, 1.571, -1.0, -1.571, 1.0, -1.571, 1.0};
    if (request->grasp_mode == "pick")
    {
      closed_target_joint_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    else if (request->grasp_mode == "grasp")
    {
      closed_target_joint_rad = {-1.65, -0.1, -0.8, -0.1, 1.0, 0.1, 0.8};
    }
  }
  else if (request->grasp_form == 1)
  {
    opened_target_joint_rad = {-0.7, 1.571, -1.0, -1.571, 1.0, -1.571, 1.0};
    if (request->grasp_mode == "pick")
    {
      closed_target_joint_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    else if (request->grasp_mode == "grasp")
    {
      closed_target_joint_rad = {-0.7, -0.1, -1.2, -0.3, 1.571, 0.1, 1.2};
    }
  }

  if (sign_multiplier == -1.0)
  {
    std::transform(
        opened_target_joint_rad.begin(),
        opened_target_joint_rad.end(),
        invert_target_joint.begin(),
        opened_target_joint_rad.begin(),
        [sign_multiplier](double rad, bool invert)
        {
          return invert ? rad * sign_multiplier : rad;
        });
    std::transform(
        closed_target_joint_rad.begin(),
        closed_target_joint_rad.end(),
        invert_target_joint.begin(),
        closed_target_joint_rad.begin(),
        [sign_multiplier](double rad, bool invert)
        {
          return invert ? rad * sign_multiplier : rad;
        });
  }

  response->success = true;
  response->message = "[SUCCESS] Finger angle calculated.";
  response->target_joint_names = target_joint_names;
  response->opened_target_joint_rad = opened_target_joint_rad;
  response->closed_target_joint_rad = closed_target_joint_rad;
  return;
}

void JointActionServer::get_head_to_coord(
    const std::shared_ptr<GetHandToTargetCoord::Request> request,
    std::shared_ptr<GetHandToTargetCoord::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->target_coord.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/head_base_link";

  // Transform to head base link from 'sobit_home/head_base_link'
  try
  {
    goal_coord = tf_buffer_->transform(
        request->target_coord, goal_coord.header.frame_id,
        tf2::durationFromSec(1.0));
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  double pan_rad, tilt_rad, target_yaw;

  if (std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) > joint_limits_["head_pan_joint"].upper)
  { // 45 degrees
    pan_rad = joint_limits_["head_pan_joint"].upper;
    target_yaw = std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) - joint_limits_["head_pan_joint"].upper;
  }
  else if (std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) < joint_limits_["head_pan_joint"].lower)
  {
    pan_rad = joint_limits_["head_pan_joint"].lower;
    target_yaw = std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) - joint_limits_["head_pan_joint"].lower;
  }
  else
  {
    pan_rad = std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x);
    target_yaw = 0.0;
  }

  if (std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) > joint_limits_["head_tilt_joint"].upper)
  { // 30 degrees
    tilt_rad = joint_limits_["head_tilt_joint"].upper;
  }
  else if (std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) < joint_limits_["head_tilt_joint"].lower)
  { // 45 degrees
    tilt_rad = joint_limits_["head_tilt_joint"].lower;
  }
  else
  {
    tilt_rad = std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2)));
  }

  if (std::abs(atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x)) < joint_limits_["head_pan_joint"].upper &&                                                                                              // 45 degrees
      std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) > joint_limits_["head_tilt_joint"].lower && // 45 degrees
      std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) < joint_limits_["head_tilt_joint"].upper)
  { // 30 degrees
    response->success = true;
    response->message = "[SUCCESS] The coord is visible.";
  }
  else
  {
    response->success = false;
    response->message = "[FAIL] The coord is out of the head rotation limit";
  }

  response->move_pose.position.x = 0.0;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->target_joint_names = {"head_pan_joint", "head_tilt_joint"};
  response->target_joint_rad = {pan_rad, tilt_rad};

  return;
}

void JointActionServer::get_head_to_tf(
    const std::shared_ptr<GetHandToTargetTF::Request> request,
    std::shared_ptr<GetHandToTargetTF::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->tf_differential.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/head_base_link";

  geometry_msgs::msg::TransformStamped goal_coord_shift;

  // Transform the target frame based on the differential tf
  try
  {
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
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not transform: %s to %s: %s", request->target_frame.c_str(), request->tf_differential.header.frame_id.c_str(), ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform: " + request->target_frame + " to: " + request->tf_differential.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // Transform to robot base from 'sobit_home/base_footprint'
  try
  {
    goal_coord = tf_buffer_->transform(
        goal_coord_shift, goal_coord.header.frame_id,
        tf2::durationFromSec(1.0));
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not transform coords to %s: %s", goal_coord.header.frame_id.c_str(), ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  double pan_rad, tilt_rad, target_yaw;

  if (std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) > joint_limits_["head_pan_joint"].upper)
  { // 45 degrees
    pan_rad = joint_limits_["head_pan_joint"].upper;
    target_yaw = std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) - joint_limits_["head_pan_joint"].upper;
  }
  else if (std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) < joint_limits_["head_pan_joint"].lower)
  {
    pan_rad = joint_limits_["head_pan_joint"].lower;
    target_yaw = std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x) - joint_limits_["head_pan_joint"].lower;
  }
  else
  {
    pan_rad = std::atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x);
    target_yaw = 0.0;
  }

  if (std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) > joint_limits_["head_tilt_joint"].upper)
  { // 30 degrees
    tilt_rad = joint_limits_["head_tilt_joint"].upper;
  }
  else if (std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) < joint_limits_["head_tilt_joint"].lower)
  { // 45 degrees
    tilt_rad = joint_limits_["head_tilt_joint"].lower;
  }
  else
  {
    tilt_rad = std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2)));
  }

  if (std::abs(atan2(goal_coord.transform.translation.y, goal_coord.transform.translation.x)) < joint_limits_["head_pan_joint"].upper &&                                                                                              // 45 degrees
      std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) > joint_limits_["head_tilt_joint"].lower && // 45 degrees
      std::atan2(goal_coord.transform.translation.z, std::sqrt(std::pow(goal_coord.transform.translation.x, 2) + std::pow(goal_coord.transform.translation.y - BodylinkToHeadtiltDZ, 2))) < joint_limits_["head_tilt_joint"].upper)
  { // 30 degrees
    response->success = true;
    response->message = "[SUCCESS] The coord is visible.";
  }
  else
  {
    response->success = false;
    response->message = "[FAIL] The coord is out of the head rotation limit";
  }

  response->move_pose.position.x = 0.0;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->target_joint_names = {"head_pan_joint", "head_tilt_joint"};
  response->target_joint_rad = {pan_rad, tilt_rad};

  return;
}

void JointActionServer::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received joint state");

  for (size_t i = 0; i < msg->name.size(); i++)
  {
    // if (msg->name[i] == "SUB JOINT") continue;  // Skip sub joints

    this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }
}

trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(
    const std::vector<std::string> &target_joint_names, // JointName
    const std::vector<double> &target_joint_rad,        // target_joint_rad
    const builtin_interfaces::msg::Duration &time_allowance,
    const std::string &group_name)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint point;

  for (size_t i = 0; i < target_joint_names.size(); i++)
  {
    // Check if the joint belongs to the specified group
    if (group_name == "arm_right" &&
        std::find(JointNamesArmRight.begin(), JointNamesArmRight.end(), target_joint_names[i]) == JointNamesArmRight.end())
    {
      continue;
    }
    else if (group_name == "arm_left" &&
             std::find(JointNamesArmLeft.begin(), JointNamesArmLeft.end(), target_joint_names[i]) == JointNamesArmLeft.end())
    {
      continue;
    }
    else if (group_name == "hand_right" &&
             std::find(JointNamesHandRight.begin(), JointNamesHandRight.end(), target_joint_names[i]) == JointNamesHandRight.end())
    {
      continue;
    }
    else if (group_name == "hand_left" &&
             std::find(JointNamesHandLeft.begin(), JointNamesHandLeft.end(), target_joint_names[i]) == JointNamesHandLeft.end())
    {
      continue;
    }
    else if (group_name == "head" &&
             std::find(JointNamesHead.begin(), JointNamesHead.end(), target_joint_names[i]) == JointNamesHead.end())
    {
      continue;
    }
    else if (group_name == "body" &&
             std::find(JointNamesBody.begin(), JointNamesBody.end(), target_joint_names[i]) == JointNamesBody.end())
    {
      continue;
    }

    joint_trajectory.joint_names.push_back(target_joint_names[i]);
    point.positions.push_back(target_joint_rad[i]);

    // Subjoint to turn opposite direction
    // if (target_joint_names[i] == "arm_shoulder_pitch_joint") {
    //   joint_trajectory.joint_names.push_back("arm_shoulder_pitch_sub_joint");
    //   point.positions.push_back(-target_joint_rad[i]);
    //   continue;
    // }
  }

  joint_trajectory.points.push_back(point);
  joint_trajectory.points[0].time_from_start = time_allowance;

  return joint_trajectory;
}
} // namespace sobit_home
