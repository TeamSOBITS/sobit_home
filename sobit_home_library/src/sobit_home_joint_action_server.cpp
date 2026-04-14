#include "sobit_home_library/sobit_home_joint_action_server.hpp"

namespace sobit_home
{
  JointActionServer::JointActionServer(const rclcpp::NodeOptions &options)
      : Node("joint_action_server", options),
        poses_(),
        curt_joint_state_(),
        kinematics_(std::make_unique<Kinematics>()),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
        urdf_loaded_(false)
  {
    rclcpp::QoS qos_profile(1);

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

    this->action_server_move_right_hand_to_pose_ = rclcpp_action::create_server<MoveToPose>(
        this, "move_right_hand_to_pose",
        std::bind(&JointActionServer::handle_move_right_hand_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointActionServer::handle_move_right_hand_to_pose_cancel, this, std::placeholders::_1),
        std::bind(&JointActionServer::handle_move_right_hand_to_pose_accepted, this, std::placeholders::_1));

    this->action_server_move_left_hand_to_pose_ = rclcpp_action::create_server<MoveToPose>(
        this, "move_left_hand_to_pose",
        std::bind(&JointActionServer::handle_move_left_hand_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointActionServer::handle_move_left_hand_to_pose_cancel, this, std::placeholders::_1),
        std::bind(&JointActionServer::handle_move_left_hand_to_pose_accepted, this, std::placeholders::_1));

    this->service_get_hand_to_coord_left_ = this->create_service<GetHandToTargetCoord>(
        "get_hand_to_coord/left", [this](const std::shared_ptr<GetHandToTargetCoord::Request> req, std::shared_ptr<GetHandToTargetCoord::Response> res)
        { get_pos_to_coord(req, res, false); });
    this->service_get_hand_to_tf_left_ = this->create_service<GetHandToTargetTF>(
        "get_hand_to_tf/left", [this](const std::shared_ptr<GetHandToTargetTF::Request> req, std::shared_ptr<GetHandToTargetTF::Response> res)
        { get_pos_to_tf(req, res, false); });
    this->service_get_hand_to_coord_right_ = this->create_service<GetHandToTargetCoord>(
        "get_hand_to_coord/right", [this](const std::shared_ptr<GetHandToTargetCoord::Request> req, std::shared_ptr<GetHandToTargetCoord::Response> res)
        { get_pos_to_coord(req, res, true); });
    this->service_get_hand_to_tf_right_ = this->create_service<GetHandToTargetTF>(
        "get_hand_to_tf/right", [this](const std::shared_ptr<GetHandToTargetTF::Request> req, std::shared_ptr<GetHandToTargetTF::Response> res)
        { get_pos_to_tf(req, res, true); });

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

    this->declare_parameter("right_hand_poses", std::vector<std::string>());
    auto right_hand_pose_names = this->get_parameter("right_hand_poses").as_string_array();

    this->declare_parameter("left_hand_poses", std::vector<std::string>());
    auto left_hand_pose_names = this->get_parameter("left_hand_poses").as_string_array();

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

    for (const auto &right_hand_pose_name : right_hand_pose_names)
    {
      this->declare_parameter(right_hand_pose_name + ".hand_right_finger_l_mcp", 0.0);
      this->declare_parameter(right_hand_pose_name + ".hand_right_finger_l_pip", 0.0);
      this->declare_parameter(right_hand_pose_name + ".hand_right_finger_l_dip", 0.0);
      this->declare_parameter(right_hand_pose_name + ".hand_right_finger_c_mcp", 0.0);
      this->declare_parameter(right_hand_pose_name + ".hand_right_finger_c_ip", 0.0);
      this->declare_parameter(right_hand_pose_name + ".hand_right_finger_r_pip", 0.0);
      this->declare_parameter(right_hand_pose_name + ".hand_right_finger_r_dip", 0.0);

      PoseParams p;
      p.pose_name = right_hand_pose_name;
      p.hand_right_finger_l_mcp = this->get_parameter(right_hand_pose_name + ".hand_right_finger_l_mcp").as_double();
      p.hand_right_finger_l_pip = this->get_parameter(right_hand_pose_name + ".hand_right_finger_l_pip").as_double();
      p.hand_right_finger_l_dip = this->get_parameter(right_hand_pose_name + ".hand_right_finger_l_dip").as_double();
      p.hand_right_finger_c_mcp = this->get_parameter(right_hand_pose_name + ".hand_right_finger_c_mcp").as_double();
      p.hand_right_finger_c_ip = this->get_parameter(right_hand_pose_name + ".hand_right_finger_c_ip").as_double();
      p.hand_right_finger_r_pip = this->get_parameter(right_hand_pose_name + ".hand_right_finger_r_pip").as_double();
      p.hand_right_finger_r_dip = this->get_parameter(right_hand_pose_name + ".hand_right_finger_r_dip").as_double();

      right_hand_poses_.push_back(p);
    }

    for (const auto &left_hand_pose_name : left_hand_pose_names)
    {
      this->declare_parameter(left_hand_pose_name + ".hand_left_finger_l_mcp", 0.0);
      this->declare_parameter(left_hand_pose_name + ".hand_left_finger_l_pip", 0.0);
      this->declare_parameter(left_hand_pose_name + ".hand_left_finger_l_dip", 0.0);
      this->declare_parameter(left_hand_pose_name + ".hand_left_finger_c_mcp", 0.0);
      this->declare_parameter(left_hand_pose_name + ".hand_left_finger_c_ip", 0.0);
      this->declare_parameter(left_hand_pose_name + ".hand_left_finger_r_pip", 0.0);
      this->declare_parameter(left_hand_pose_name + ".hand_left_finger_r_dip", 0.0);

      PoseParams p;
      p.pose_name = left_hand_pose_name;
      p.hand_left_finger_l_mcp = this->get_parameter(left_hand_pose_name + ".hand_left_finger_l_mcp").as_double();
      p.hand_left_finger_l_pip = this->get_parameter(left_hand_pose_name + ".hand_left_finger_l_pip").as_double();
      p.hand_left_finger_l_dip = this->get_parameter(left_hand_pose_name + ".hand_left_finger_l_dip").as_double();
      p.hand_left_finger_c_mcp = this->get_parameter(left_hand_pose_name + ".hand_left_finger_c_mcp").as_double();
      p.hand_left_finger_c_ip = this->get_parameter(left_hand_pose_name + ".hand_left_finger_c_ip").as_double();
      p.hand_left_finger_r_pip = this->get_parameter(left_hand_pose_name + ".hand_left_finger_r_pip").as_double();
      p.hand_left_finger_r_dip = this->get_parameter(left_hand_pose_name + ".hand_left_finger_r_dip").as_double();

      left_hand_poses_.push_back(p);
    }


    RCLCPP_INFO(this->get_logger(), "JointActionServer has been initialized.");
  }

  JointActionServer::~JointActionServer() {}

  rclcpp_action::GoalResponse JointActionServer::handle_move_joints_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoint::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
  rclcpp_action::GoalResponse JointActionServer::handle_move_to_pose_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
  rclcpp_action::GoalResponse JointActionServer::handle_move_right_hand_to_pose_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
  rclcpp_action::GoalResponse JointActionServer::handle_move_left_hand_to_pose_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }

  rclcpp_action::CancelResponse JointActionServer::handle_move_joints_cancel(const std::shared_ptr<GoalHandleMoveJoints>) { return rclcpp_action::CancelResponse::ACCEPT; }
  rclcpp_action::CancelResponse JointActionServer::handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose>) { return rclcpp_action::CancelResponse::ACCEPT; }
  rclcpp_action::CancelResponse JointActionServer::handle_move_right_hand_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose>) { return rclcpp_action::CancelResponse::ACCEPT; }
  rclcpp_action::CancelResponse JointActionServer::handle_move_left_hand_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose>) { return rclcpp_action::CancelResponse::ACCEPT; }

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

  void JointActionServer::handle_move_right_hand_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Accepted move_right_hand_to_pose goal request");
    std::thread{std::bind(&JointActionServer::exe_move_right_hand_to_pose, this, std::placeholders::_1), goal_handle}.detach();
  }

  void JointActionServer::handle_move_left_hand_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Accepted move_left_hand_to_pose goal request");
    std::thread{std::bind(&JointActionServer::exe_move_left_hand_to_pose, this, std::placeholders::_1), goal_handle}.detach();
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

      auto feedback = std::make_shared<MoveJoint::Feedback>();
      feedback->current_joint_names = goal->target_joint_names;
      for (const auto &name : goal->target_joint_names)
      {
        feedback->current_joint_rad.push_back(curt_joint_state_[name]);
      }
      goal_handle->publish_feedback(feedback);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  void JointActionServer::exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    auto it = std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams &p)
                           { return p.pose_name == goal->pose_name; });
    if (it == poses_.end())
    {
      goal_handle->abort(result);
      return;
    }

    std::vector<std::string> names;
    std::vector<double> rads;
    auto add = [&](const std::vector<std::string> &n, const std::vector<double> &r)
    {
      names.insert(names.end(), n.begin(), n.end());
      rads.insert(rads.end(), r.begin(), r.end());
    };

    add(JointNamesArmRight, {it->arm_right_shoulder_tilt, it->arm_right_upper_roll, it->arm_right_upper_flex, it->arm_right_elbow, it->arm_right_wrist_tilt, it->arm_right_wrist_roll});
    add(JointNamesHandRight, {it->hand_right_finger_l_mcp, it->hand_right_finger_l_pip, it->hand_right_finger_l_dip, it->hand_right_finger_c_mcp, it->hand_right_finger_c_ip, it->hand_right_finger_r_pip, it->hand_right_finger_r_dip});
    add(JointNamesArmLeft, {it->arm_left_shoulder_tilt, it->arm_left_upper_roll, it->arm_left_upper_flex, it->arm_left_elbow, it->arm_left_wrist_tilt, it->arm_left_wrist_roll});
    add(JointNamesHandLeft, {it->hand_left_finger_l_mcp, it->hand_left_finger_l_pip, it->hand_left_finger_l_dip, it->hand_left_finger_c_mcp, it->hand_left_finger_c_ip, it->hand_left_finger_r_pip, it->hand_left_finger_r_dip});
    add(JointNamesBody, {it->body_lift});
    add(JointNamesHead, {it->head_pan, it->head_tilt});

    auto publish_group = [&](auto &pub, const std::string &grp)
    {
      auto traj = set_joints(names, rads, goal->time_allowance, grp);
      if (!traj.joint_names.empty())
        pub->publish(traj);
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
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    result->success = true;
    goal_handle->succeed(result);
  }

  // right hand
  void JointActionServer::exe_move_right_hand_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    auto it = std::find_if(right_hand_poses_.begin(), right_hand_poses_.end(), [&](const PoseParams &p)
                           { return p.pose_name == goal->pose_name; });
    if (it == right_hand_poses_.end())
    {
      goal_handle->abort(result);
      return;
    }

    std::vector<std::string> names;
    std::vector<double> rads;
    auto add = [&](const std::vector<std::string> &n, const std::vector<double> &r)
    {
      names.insert(names.end(), n.begin(), n.end());
      rads.insert(rads.end(), r.begin(), r.end());
    };

    add(JointNamesHandRight, {it->hand_right_finger_l_mcp, it->hand_right_finger_l_pip, it->hand_right_finger_l_dip, it->hand_right_finger_c_mcp, it->hand_right_finger_c_ip, it->hand_right_finger_r_pip, it->hand_right_finger_r_dip});

    auto publish_group = [&](auto &pub, const std::string &grp)
    {
      auto traj = set_joints(names, rads, goal->time_allowance, grp);
      if (!traj.joint_names.empty())
        pub->publish(traj);
    };

    publish_group(pub_right_hand_joint_control_, "hand_right");

    auto start_time = this->now();
    while (this->now() - start_time < goal->time_allowance)
    {
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    result->success = true;
    goal_handle->succeed(result);
  }

  // left hand
  void JointActionServer::exe_move_left_hand_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    auto it = std::find_if(left_hand_poses_.begin(), left_hand_poses_.end(), [&](const PoseParams &p)
                           { return p.pose_name == goal->pose_name; });
    if (it == left_hand_poses_.end())
    {
      goal_handle->abort(result);
      return;
    }

    std::vector<std::string> names;
    std::vector<double> rads;
    auto add = [&](const std::vector<std::string> &n, const std::vector<double> &r)
    {
      names.insert(names.end(), n.begin(), n.end());
      rads.insert(rads.end(), r.begin(), r.end());
    };

    add(JointNamesHandLeft, {it->hand_left_finger_l_mcp, it->hand_left_finger_l_pip, it->hand_left_finger_l_dip, it->hand_left_finger_c_mcp, it->hand_left_finger_c_ip, it->hand_left_finger_r_pip, it->hand_left_finger_r_dip});

    auto publish_group = [&](auto &pub, const std::string &grp)
    {
      auto traj = set_joints(names, rads, goal->time_allowance, grp);
      if (!traj.joint_names.empty())
        pub->publish(traj);
    };

    publish_group(pub_left_hand_joint_control_, "hand_left");

    auto start_time = this->now();
    while (this->now() - start_time < goal->time_allowance)
    {
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    result->success = true;
    goal_handle->succeed(result);
  }

  void JointActionServer::get_pos_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request, std::shared_ptr<GetHandToTargetCoord::Response> response, bool is_right)
  {
    geometry_msgs::msg::TransformStamped goal_in_base;
    geometry_msgs::msg::TransformStamped goal_in_lift;
    std::string base_frame = "sobit_home/base_footprint";
    std::string lift_frame = "sobit_home/body_lift_link";

    try
    {
      goal_in_base = tf_buffer_->transform(request->target_coord, base_frame, tf2::durationFromSec(1.0));
      goal_in_lift = tf_buffer_->transform(request->target_coord, lift_frame, tf2::durationFromSec(1.0));
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
      response->success = false;
      response->message = "TF error: " + std::string(ex.what());
      return;
    }

    auto rads = kinematics_->inverse_kinematics(goal_in_lift, is_right);

    if (rads.empty())
    {
      response->success = false;
      response->message = "Target unreachable.";
      return;
    }

    response->move_pose = kinematics_->forward_kinematics(rads, goal_in_base, is_right);
    response->target_joint_names = is_right ? JointNamesArmRight : JointNamesArmLeft;
    response->target_joint_rad = rads;
    response->success = true;
    response->message = "Success: Target reached.";
  }

  void JointActionServer::get_pos_to_tf(const std::shared_ptr<GetHandToTargetTF::Request> request, std::shared_ptr<GetHandToTargetTF::Response> response, bool is_right)
  {
    geometry_msgs::msg::TransformStamped goal_in_base;
    geometry_msgs::msg::TransformStamped goal_in_lift;
    std::string base_frame = "sobit_home/base_footprint";
    std::string lift_frame = "sobit_home/body_lift_link";

    try
    {
      goal_in_base = tf_buffer_->lookupTransform(base_frame, request->target_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
      goal_in_lift = tf_buffer_->lookupTransform(lift_frame, request->target_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      response->success = false;
      response->message = "TF error: " + std::string(ex.what());
      return;
    }

    goal_in_base.transform.translation.x += request->tf_differential.transform.translation.x;
    goal_in_base.transform.translation.y += request->tf_differential.transform.translation.y;
    goal_in_base.transform.translation.z += request->tf_differential.transform.translation.z;

    goal_in_lift.transform.translation.x += request->tf_differential.transform.translation.x;
    goal_in_lift.transform.translation.y += request->tf_differential.transform.translation.y;
    goal_in_lift.transform.translation.z += request->tf_differential.transform.translation.z;

    auto rads = kinematics_->inverse_kinematics(goal_in_lift, is_right);

    if (rads.empty())
    {
      response->success = false;
      response->message = "Target unreachable with offset.";
      return;
    }

    response->move_pose = kinematics_->forward_kinematics(rads, goal_in_base, is_right);
    response->target_joint_names = is_right ? JointNamesArmRight : JointNamesArmLeft;
    response->target_joint_rad = rads;
    response->success = true;
    response->message = "Success: TF target reached.";
  }

  void JointActionServer::get_head_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request, std::shared_ptr<GetHandToTargetCoord::Response> response)
  {
    geometry_msgs::msg::TransformStamped goal_head;
    try
    {
      goal_head = tf_buffer_->transform(request->target_coord, std::string(this->get_namespace()).substr(1) + "/head_base_link", tf2::durationFromSec(1.0));
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      response->success = false;
      return;
    }

    auto rads = kinematics_->look_at(goal_head);
    response->target_joint_names = JointNamesHead;
    response->target_joint_rad = rads;
    response->success = true;
  }

  void JointActionServer::get_head_to_tf(const std::shared_ptr<GetHandToTargetTF::Request> request, std::shared_ptr<GetHandToTargetTF::Response> response)
  {
    geometry_msgs::msg::TransformStamped goal_head;
    try
    {
      goal_head = tf_buffer_->lookupTransform(std::string(this->get_namespace()).substr(1) + "/head_base_link", request->target_frame, tf2::TimePointZero);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      response->success = false;
      return;
    }

    auto rads = kinematics_->look_at(goal_head);
    response->target_joint_names = JointNamesHead;
    response->target_joint_rad = rads;
    response->success = true;
  }

  void JointActionServer::serve_get_finger_angle(const std::shared_ptr<GetFingerAngle::Request> request, std::shared_ptr<GetFingerAngle::Response> response)
  {
    response->target_joint_names = request->is_right ? JointNamesHandRight : JointNamesHandLeft;
    if (request->grasp_form == 0)
    {
      response->opened_target_joint_rad = {-1.65, 1.571, -1.0, -1.571, 1.0, -1.571, 1.0};
      response->closed_target_joint_rad = (request->grasp_mode == "pick") ? std::vector<double>(7, 0.0) : std::vector<double>{-1.65, -0.1, -0.8, -0.1, 1.0, 0.1, 0.8};
    }
    response->success = true;
  }

  void JointActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); i++)
      this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }

  trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(const std::vector<std::string> &names, const std::vector<double> &rads, const builtin_interfaces::msg::Duration &dur, const std::string &grp)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    auto is_in = [&](const std::vector<std::string> &v, const std::string &n)
    { return std::find(v.begin(), v.end(), n) != v.end(); };
    const std::vector<std::string> *target_v = nullptr;

    if (grp == "arm_left")
      target_v = &JointNamesArmLeft;
    else if (grp == "arm_right")
      target_v = &JointNamesArmRight;
    else if (grp == "hand_left")
      target_v = &JointNamesHandLeft;
    else if (grp == "hand_right")
      target_v = &JointNamesHandRight;
    else if (grp == "head")
      target_v = &JointNamesHead;
    else if (grp == "body")
      target_v = &JointNamesBody;

    if (target_v)
    {
      for (size_t i = 0; i < names.size(); i++)
      {
        if (is_in(*target_v, names[i]))
        {
          traj.joint_names.push_back(names[i]);
          p.positions.push_back(rads[i]);
        }
      }
    }
    if (!traj.joint_names.empty())
    {
      p.time_from_start = dur;
      traj.points.push_back(p);
    }
    return traj;
  }

  void JointActionServer::load_joint_limits()
  {
    if (urdf_loaded_)
      return;
    auto param = this->get_parameter("robot_description");
    std::string xml = param.as_string();
    if (xml.empty())
      return;
    if (!parse_urdf_limits(xml))
      return;
    urdf_loaded_ = true;
    if (urdf_timer_)
      urdf_timer_->cancel();
  }

  bool JointActionServer::parse_urdf_limits(const std::string &xml)
  {
    urdf::Model model;
    if (!model.initString(xml))
      return false;
    for (auto const &[name, joint] : model.joints_)
    {
      if (joint->limits)
      {
        Limit l;
        l.lower = joint->limits->lower;
        l.upper = joint->limits->upper;
        l.velocity = joint->limits->velocity;
        l.effort = joint->limits->effort;
        l.has = true;
        joint_limits_[name] = l;
      }
    }
    return true;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::JointActionServer)