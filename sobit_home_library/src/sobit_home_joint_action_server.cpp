#include "sobit_home_library/sobit_home_joint_action_server.hpp"

namespace sobit_home
{
  JointActionServer::JointActionServer(const rclcpp::NodeOptions & options)
      : Node("joint_action_server", options),
        poses_(),
        curt_joint_state_(),
        kinematics_(std::make_unique<Kinematics>()),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
        urdf_loaded_(false)
  {
    rclcpp::QoS qos_profile(1);

    action_server_move_joints_ = rclcpp_action::create_server<MoveJoint>(
      this, "move_joint",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoint::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleMoveJoints>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMoveJoints> goal_handle) {
        RCLCPP_INFO(get_logger(), "Accepted move_joints goal request");
        std::thread{[this, goal_handle]() { exe_move_joints(goal_handle); }}.detach();
      });

    action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        RCLCPP_INFO(get_logger(), "Accepted move_to_pose goal request");
        std::thread{[this, goal_handle]() { exe_move_to_pose(goal_handle); }}.detach();
      });

    action_server_move_right_hand_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_right_hand_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        RCLCPP_INFO(get_logger(), "Accepted move_right_hand_to_pose goal request");
        std::thread{[this, goal_handle]() { exe_move_right_hand_to_pose(goal_handle); }}.detach();
      });

    action_server_move_left_hand_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_left_hand_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        RCLCPP_INFO(get_logger(), "Accepted move_left_hand_to_pose goal request");
        std::thread{[this, goal_handle]() { exe_move_left_hand_to_pose(goal_handle); }}.detach();
      });

    service_get_hand_to_coord_left_ = create_service<GetHandToTargetCoord>(
      "get_hand_to_coord/left",
      [this](
        const std::shared_ptr<GetHandToTargetCoord::Request> req,
        std::shared_ptr<GetHandToTargetCoord::Response> res) {
        get_pos_to_coord(req, res, false);
      });

    service_get_hand_to_tf_left_ = create_service<GetHandToTargetTF>(
      "get_hand_to_tf/left",
      [this](
        const std::shared_ptr<GetHandToTargetTF::Request> req,
        std::shared_ptr<GetHandToTargetTF::Response> res) {
        get_pos_to_tf(req, res, false);
      });

    service_get_hand_to_coord_right_ = create_service<GetHandToTargetCoord>(
      "get_hand_to_coord/right",
      [this](
        const std::shared_ptr<GetHandToTargetCoord::Request> req,
        std::shared_ptr<GetHandToTargetCoord::Response> res) {
        get_pos_to_coord(req, res, true);
      });

    service_get_hand_to_tf_right_ = create_service<GetHandToTargetTF>(
      "get_hand_to_tf/right",
      [this](
        const std::shared_ptr<GetHandToTargetTF::Request> req,
        std::shared_ptr<GetHandToTargetTF::Response> res) {
        get_pos_to_tf(req, res, true);
      });

    service_get_head_to_coord_ = create_service<GetHandToTargetCoord>(
      "get_head_to_coord",
      [this](
        const std::shared_ptr<GetHandToTargetCoord::Request> req,
        std::shared_ptr<GetHandToTargetCoord::Response> res) {
        get_head_to_coord(req, res);
      });

    service_get_head_to_tf_ = create_service<GetHandToTargetTF>(
      "get_head_to_tf",
      [this](
        const std::shared_ptr<GetHandToTargetTF::Request> req,
        std::shared_ptr<GetHandToTargetTF::Response> res) {
        get_head_to_tf(req, res);
      });

    service_server_get_finger_angle_ = create_service<GetFingerAngle>(
      "get_finger_angle",
      [this](
        const std::shared_ptr<GetFingerAngle::Request> req,
        std::shared_ptr<GetFingerAngle::Response> res) {
        serve_get_finger_angle(req, res);
      });

    sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      });

    pub_left_arm_joint_control_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_left_position_controller/joint_trajectory", qos_profile);
    pub_left_hand_joint_control_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "hand_left_position_controller/joint_trajectory", qos_profile);
    pub_right_arm_joint_control_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_right_position_controller/joint_trajectory", qos_profile);
    pub_right_hand_joint_control_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "hand_right_position_controller/joint_trajectory", qos_profile);
    pub_body_joint_control_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "body_position_controller/joint_trajectory", qos_profile);
    pub_head_joint_control_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "head_position_controller/joint_trajectory", qos_profile);

    declare_parameter("robot_description", "");
    urdf_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { load_joint_limits(); });

    declare_parameter("poses", std::vector<std::string>());
    declare_parameter("right_hand_poses", std::vector<std::string>());
    declare_parameter("left_hand_poses", std::vector<std::string>());

    const auto pose_names = get_parameter("poses").as_string_array();
    const auto right_hand_pose_names = get_parameter("right_hand_poses").as_string_array();
    const auto left_hand_pose_names = get_parameter("left_hand_poses").as_string_array();

    for (const auto & pose_name : pose_names) {
      declare_parameter(pose_name + ".arm_right_shoulder_tilt", 0.0);
      declare_parameter(pose_name + ".arm_right_upper_roll", 0.0);
      declare_parameter(pose_name + ".arm_right_upper_flex", 0.0);
      declare_parameter(pose_name + ".arm_right_elbow", 0.0);
      declare_parameter(pose_name + ".arm_right_lower_flex", 0.0);
      declare_parameter(pose_name + ".arm_right_wrist_tilt", 0.0);
      declare_parameter(pose_name + ".arm_right_wrist_roll", 0.0);

      declare_parameter(pose_name + ".hand_right_finger_l_mcp", 0.0);
      declare_parameter(pose_name + ".hand_right_finger_l_pip", 0.0);
      declare_parameter(pose_name + ".hand_right_finger_l_dip", 0.0);
      declare_parameter(pose_name + ".hand_right_finger_c_mcp", 0.0);
      declare_parameter(pose_name + ".hand_right_finger_c_ip", 0.0);
      declare_parameter(pose_name + ".hand_right_finger_r_pip", 0.0);
      declare_parameter(pose_name + ".hand_right_finger_r_dip", 0.0);

      declare_parameter(pose_name + ".arm_left_shoulder_tilt", 0.0);
      declare_parameter(pose_name + ".arm_left_upper_roll", 0.0);
      declare_parameter(pose_name + ".arm_left_upper_flex", 0.0);
      declare_parameter(pose_name + ".arm_left_elbow", 0.0);
      declare_parameter(pose_name + ".arm_left_lower_flex", 0.0);
      declare_parameter(pose_name + ".arm_left_wrist_tilt", 0.0);
      declare_parameter(pose_name + ".arm_left_wrist_roll", 0.0);

      declare_parameter(pose_name + ".hand_left_finger_l_mcp", 0.0);
      declare_parameter(pose_name + ".hand_left_finger_l_pip", 0.0);
      declare_parameter(pose_name + ".hand_left_finger_l_dip", 0.0);
      declare_parameter(pose_name + ".hand_left_finger_c_mcp", 0.0);
      declare_parameter(pose_name + ".hand_left_finger_c_ip", 0.0);
      declare_parameter(pose_name + ".hand_left_finger_r_pip", 0.0);
      declare_parameter(pose_name + ".hand_left_finger_r_dip", 0.0);

      declare_parameter(pose_name + ".body_lift", 0.0);
      declare_parameter(pose_name + ".head_pan", 0.0);
      declare_parameter(pose_name + ".head_tilt", 0.0);

      auto get = [&](const std::string & key) {
        return get_parameter(pose_name + "." + key).as_double();
      };

      PoseParams p;
      p.pose_name = pose_name;

      p.arm_right_shoulder_tilt = get("arm_right_shoulder_tilt");
      p.arm_right_upper_roll    = get("arm_right_upper_roll");
      p.arm_right_upper_flex    = get("arm_right_upper_flex");
      p.arm_right_elbow         = get("arm_right_elbow");
      p.arm_right_lower_flex    = get("arm_right_lower_flex");
      p.arm_right_wrist_tilt    = get("arm_right_wrist_tilt");
      p.arm_right_wrist_roll    = get("arm_right_wrist_roll");

      p.hand_right_finger_l_mcp = get("hand_right_finger_l_mcp");
      p.hand_right_finger_l_pip = get("hand_right_finger_l_pip");
      p.hand_right_finger_l_dip = get("hand_right_finger_l_dip");
      p.hand_right_finger_c_mcp = get("hand_right_finger_c_mcp");
      p.hand_right_finger_c_ip  = get("hand_right_finger_c_ip");
      p.hand_right_finger_r_pip = get("hand_right_finger_r_pip");
      p.hand_right_finger_r_dip = get("hand_right_finger_r_dip");

      p.arm_left_shoulder_tilt = get("arm_left_shoulder_tilt");
      p.arm_left_upper_roll    = get("arm_left_upper_roll");
      p.arm_left_upper_flex    = get("arm_left_upper_flex");
      p.arm_left_elbow         = get("arm_left_elbow");
      p.arm_left_lower_flex    = get("arm_left_lower_flex");
      p.arm_left_wrist_tilt    = get("arm_left_wrist_tilt");
      p.arm_left_wrist_roll    = get("arm_left_wrist_roll");

      p.hand_left_finger_l_mcp = get("hand_left_finger_l_mcp");
      p.hand_left_finger_l_pip = get("hand_left_finger_l_pip");
      p.hand_left_finger_l_dip = get("hand_left_finger_l_dip");
      p.hand_left_finger_c_mcp = get("hand_left_finger_c_mcp");
      p.hand_left_finger_c_ip  = get("hand_left_finger_c_ip");
      p.hand_left_finger_r_pip = get("hand_left_finger_r_pip");
      p.hand_left_finger_r_dip = get("hand_left_finger_r_dip");

      p.body_lift  = get("body_lift");
      p.head_pan   = get("head_pan");
      p.head_tilt  = get("head_tilt");

      poses_.push_back(p);
    }

    for (const auto & name : right_hand_pose_names) {
      declare_parameter(name + ".hand_right_finger_l_mcp", 0.0);
      declare_parameter(name + ".hand_right_finger_l_pip", 0.0);
      declare_parameter(name + ".hand_right_finger_l_dip", 0.0);
      declare_parameter(name + ".hand_right_finger_c_mcp", 0.0);
      declare_parameter(name + ".hand_right_finger_c_ip", 0.0);
      declare_parameter(name + ".hand_right_finger_r_pip", 0.0);
      declare_parameter(name + ".hand_right_finger_r_dip", 0.0);

      auto get = [&](const std::string & key) {
        return get_parameter(name + "." + key).as_double();
      };

      PoseParams p;
      p.pose_name               = name;
      p.hand_right_finger_l_mcp = get("hand_right_finger_l_mcp");
      p.hand_right_finger_l_pip = get("hand_right_finger_l_pip");
      p.hand_right_finger_l_dip = get("hand_right_finger_l_dip");
      p.hand_right_finger_c_mcp = get("hand_right_finger_c_mcp");
      p.hand_right_finger_c_ip  = get("hand_right_finger_c_ip");
      p.hand_right_finger_r_pip = get("hand_right_finger_r_pip");
      p.hand_right_finger_r_dip = get("hand_right_finger_r_dip");

      right_hand_poses_.push_back(p);
    }

    for (const auto & name : left_hand_pose_names) {
      declare_parameter(name + ".hand_left_finger_l_mcp", 0.0);
      declare_parameter(name + ".hand_left_finger_l_pip", 0.0);
      declare_parameter(name + ".hand_left_finger_l_dip", 0.0);
      declare_parameter(name + ".hand_left_finger_c_mcp", 0.0);
      declare_parameter(name + ".hand_left_finger_c_ip", 0.0);
      declare_parameter(name + ".hand_left_finger_r_pip", 0.0);
      declare_parameter(name + ".hand_left_finger_r_dip", 0.0);

      auto get = [&](const std::string & key) {
        return get_parameter(name + "." + key).as_double();
      };

      PoseParams p;
      p.pose_name              = name;
      p.hand_left_finger_l_mcp = get("hand_left_finger_l_mcp");
      p.hand_left_finger_l_pip = get("hand_left_finger_l_pip");
      p.hand_left_finger_l_dip = get("hand_left_finger_l_dip");
      p.hand_left_finger_c_mcp = get("hand_left_finger_c_mcp");
      p.hand_left_finger_c_ip  = get("hand_left_finger_c_ip");
      p.hand_left_finger_r_pip = get("hand_left_finger_r_pip");
      p.hand_left_finger_r_dip = get("hand_left_finger_r_dip");

      left_hand_poses_.push_back(p);
    }

    declare_parameter("enable_tf_prefix", false);

    RCLCPP_INFO(get_logger(), "JointActionServer has been initialized.");
  }

  JointActionServer::~JointActionServer() {}

  void JointActionServer::exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveJoint::Result>();

    if (goal->target_joint_names.size() != goal->target_joint_rad.size()) {
      RCLCPP_ERROR(get_logger(), "Joint names and rad size mismatch.");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    auto publish_group = [&](auto & pub, const std::string & grp) {
      auto traj = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, grp);
      if (!traj.joint_names.empty()) {
        try {
          pub->publish(traj);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Publish failed in %s: %s", grp.c_str(), e.what());
        }
      }
    };

    publish_group(pub_left_arm_joint_control_,   "arm_left");
    publish_group(pub_right_arm_joint_control_,  "arm_right");
    publish_group(pub_left_hand_joint_control_,  "hand_left");
    publish_group(pub_right_hand_joint_control_, "hand_right");
    publish_group(pub_body_joint_control_,       "body");
    publish_group(pub_head_joint_control_,       "head");

    auto start_time = now();
    while (now() - start_time < goal->time_allowance) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      auto feedback = std::make_shared<MoveJoint::Feedback>();
      feedback->current_joint_names = goal->target_joint_names;
      for (const auto & name : goal->target_joint_names) {
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

    auto it = std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams & p) {
      return p.pose_name == goal->pose_name;
    });
    if (it == poses_.end()) {
      RCLCPP_WARN(get_logger(), "Pose '%s' not found.", goal->pose_name.c_str());
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    std::vector<std::string> names;
    std::vector<double> rads;
    auto add = [&](const std::vector<std::string> & n, const std::vector<double> & r) {
      names.insert(names.end(), n.begin(), n.end());
      rads.insert(rads.end(), r.begin(), r.end());
    };

    add(JointNamesArmRight, {it->arm_right_shoulder_tilt, it->arm_right_upper_roll, it->arm_right_upper_flex, it->arm_right_elbow, it->arm_right_lower_flex, it->arm_right_wrist_tilt, it->arm_right_wrist_roll});
    add(JointNamesArmLeft,  {it->arm_left_shoulder_tilt,  it->arm_left_upper_roll,  it->arm_left_upper_flex,  it->arm_left_elbow, it->arm_left_lower_flex,  it->arm_left_wrist_tilt,  it->arm_left_wrist_roll});
    add(JointNamesBody,     {it->body_lift});
    add(JointNamesHead,     {it->head_pan, it->head_tilt});

    auto publish_group = [&](auto & pub, const std::string & grp) {
      auto traj = set_joints(names, rads, goal->time_allowance, grp);
      if (!traj.joint_names.empty()) {
        pub->publish(traj);
      }
    };

    publish_group(pub_left_arm_joint_control_,  "arm_left");
    publish_group(pub_right_arm_joint_control_, "arm_right");
    publish_group(pub_body_joint_control_,      "body");
    publish_group(pub_head_joint_control_,      "head");

    auto start_time = now();
    while (now() - start_time < goal->time_allowance) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  void JointActionServer::exe_move_right_hand_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    auto it = std::find_if(right_hand_poses_.begin(), right_hand_poses_.end(), [&](const PoseParams & p) {
      return p.pose_name == goal->pose_name;
    });
    if (it == right_hand_poses_.end()) {
      RCLCPP_WARN(get_logger(), "Right hand pose '%s' not found.", goal->pose_name.c_str());
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    std::vector<std::string> names;
    std::vector<double> rads;
    auto add = [&](const std::vector<std::string> & n, const std::vector<double> & r) {
      names.insert(names.end(), n.begin(), n.end());
      rads.insert(rads.end(), r.begin(), r.end());
    };

    add(JointNamesHandRight, {it->hand_right_finger_l_mcp, it->hand_right_finger_l_pip, it->hand_right_finger_l_dip, it->hand_right_finger_c_mcp, it->hand_right_finger_c_ip, it->hand_right_finger_r_pip, it->hand_right_finger_r_dip});

    auto traj = set_joints(names, rads, goal->time_allowance, "hand_right");
    if (!traj.joint_names.empty()) {
      pub_right_hand_joint_control_->publish(traj);
    }

    auto start_time = now();
    while (now() - start_time < goal->time_allowance) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  void JointActionServer::exe_move_left_hand_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    auto it = std::find_if(left_hand_poses_.begin(), left_hand_poses_.end(), [&](const PoseParams & p) {
      return p.pose_name == goal->pose_name;
    });
    if (it == left_hand_poses_.end()) {
      RCLCPP_WARN(get_logger(), "Left hand pose '%s' not found.", goal->pose_name.c_str());
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    std::vector<std::string> names;
    std::vector<double> rads;
    auto add = [&](const std::vector<std::string> & n, const std::vector<double> & r) {
      names.insert(names.end(), n.begin(), n.end());
      rads.insert(rads.end(), r.begin(), r.end());
    };

    add(JointNamesHandLeft, {it->hand_left_finger_l_mcp, it->hand_left_finger_l_pip, it->hand_left_finger_l_dip, it->hand_left_finger_c_mcp, it->hand_left_finger_c_ip, it->hand_left_finger_r_pip, it->hand_left_finger_r_dip});

    auto traj = set_joints(names, rads, goal->time_allowance, "hand_left");
    if (!traj.joint_names.empty()) {
      pub_left_hand_joint_control_->publish(traj);
    }

    auto start_time = now();
    while (now() - start_time < goal->time_allowance) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  void JointActionServer::get_pos_to_coord(
    const std::shared_ptr<GetHandToTargetCoord::Request> request,
    std::shared_ptr<GetHandToTargetCoord::Response> response,
    bool is_right)
  {
    const std::string base_frame = get_tf_frame("base_footprint");
    const std::string lift_frame = get_tf_frame("body_lift_link");

    geometry_msgs::msg::TransformStamped goal_in_base;
    geometry_msgs::msg::TransformStamped goal_in_lift;
    try {
      goal_in_base = tf_buffer_->transform(request->target_coord, base_frame, tf2::durationFromSec(1.0));
      goal_in_lift = tf_buffer_->transform(request->target_coord, lift_frame, tf2::durationFromSec(1.0));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "TF transform failed: %s", ex.what());
      response->success = false;
      response->message = "TF error: " + std::string(ex.what());
      return;
    }

    auto rads = kinematics_->inverse_kinematics(goal_in_lift, is_right);

    // Always compute move_pose: base shift needed to reach (or already at) target
    response->move_pose = kinematics_->forward_kinematics(rads, goal_in_base, goal_in_lift, is_right);

    if (rads.empty())
    {
      response->success = false;
      const double dx = response->move_pose.position.x;
      const double dz = response->move_pose.position.z;
      std::string msg = "Target unreachable:";
      if (std::abs(dz) > 0.001)
        msg += " adjust lift " + std::to_string(std::abs(dz)) + " m " +
               (dz > 0 ? "up" : "down") + ";";
      if (std::abs(dx) > 0.001)
        msg += " drive base " + std::to_string(std::abs(dx)) + " m " +
               (dx > 0 ? "forward" : "backward") + ";";
      if (std::abs(dx) < 0.001 && std::abs(dz) < 0.001)
        msg += " out of workspace.";
      response->message = msg;
      return;
    }

    response->target_joint_names = is_right ? JointNamesArmRight : JointNamesArmLeft;
    response->target_joint_rad   = rads;
    response->success            = true;
    response->message            = "Success: Target reached.";
  }

  void JointActionServer::get_pos_to_tf(
    const std::shared_ptr<GetHandToTargetTF::Request> request,
    std::shared_ptr<GetHandToTargetTF::Response> response,
    bool is_right)
  {
    const std::string base_frame = get_tf_frame("base_footprint");
    const std::string lift_frame = get_tf_frame("body_lift_link");

    geometry_msgs::msg::TransformStamped goal_in_base;
    geometry_msgs::msg::TransformStamped goal_in_lift;
    try {
      goal_in_base = tf_buffer_->lookupTransform(base_frame, request->target_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
      goal_in_lift = tf_buffer_->lookupTransform(lift_frame, request->target_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
      response->success = false;
      response->message = "TF error: " + std::string(ex.what());
      return;
    }

    const auto & diff = request->tf_differential.transform.translation;
    goal_in_base.transform.translation.x += diff.x;
    goal_in_base.transform.translation.y += diff.y;
    goal_in_base.transform.translation.z += diff.z;
    goal_in_lift.transform.translation.x += diff.x;
    goal_in_lift.transform.translation.y += diff.y;
    goal_in_lift.transform.translation.z += diff.z;

    const auto rads = kinematics_->inverse_kinematics(goal_in_lift, is_right);
    if (rads.empty()) {
      response->success = false;
      response->message = "Target unreachable with offset.";
      return;
    }

    response->move_pose          = kinematics_->forward_kinematics(rads, goal_in_base, goal_in_lift, is_right);
    response->target_joint_names = is_right ? JointNamesArmRight : JointNamesArmLeft;
    response->target_joint_rad   = rads;
    response->success            = true;
    response->message            = "Success: TF target reached.";
  }

  void JointActionServer::get_head_to_coord(
    const std::shared_ptr<GetHandToTargetCoord::Request> request,
    std::shared_ptr<GetHandToTargetCoord::Response> response)
  {
    geometry_msgs::msg::TransformStamped goal_head;
    try {
      goal_head = tf_buffer_->transform(request->target_coord, get_tf_frame("head_base_link"), tf2::durationFromSec(1.0));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
      response->success = false;
      return;
    }

    response->target_joint_names = JointNamesHead;
    response->target_joint_rad   = kinematics_->look_at(goal_head);
    response->success            = true;
  }

  void JointActionServer::get_head_to_tf(
    const std::shared_ptr<GetHandToTargetTF::Request> request,
    std::shared_ptr<GetHandToTargetTF::Response> response)
  {
    geometry_msgs::msg::TransformStamped goal_head;
    try {
      goal_head = tf_buffer_->lookupTransform(get_tf_frame("head_base_link"), request->target_frame, tf2::TimePointZero);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
      response->success = false;
      return;
    }

    response->target_joint_names = JointNamesHead;
    response->target_joint_rad   = kinematics_->look_at(goal_head);
    response->success            = true;
  }

  void JointActionServer::serve_get_finger_angle(
    const std::shared_ptr<GetFingerAngle::Request> request,
    std::shared_ptr<GetFingerAngle::Response> response)
  {
    response->target_joint_names = request->is_right ? JointNamesHandRight : JointNamesHandLeft;
    if (request->grasp_form == 0) {
      response->opened_target_joint_rad = {-1.65, 1.571, -1.0, -1.571, 1.0, -1.571, 1.0};
      response->closed_target_joint_rad = (request->grasp_mode == "pick")
        ? std::vector<double>(7, 0.0)
        : std::vector<double>{-1.65, -0.1, -0.8, -0.1, 1.0, 0.1, 0.8};
    }
    response->success = true;
  }

  void JointActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      curt_joint_state_[msg->name[i]] = msg->position[i];
    }
  }

  trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(
    const std::vector<std::string> & names,
    const std::vector<double> & rads,
    const builtin_interfaces::msg::Duration & dur,
    const std::string & grp)
  {
    static const std::unordered_map<std::string, const std::vector<std::string> *> group_map = {
      {"arm_left",   &JointNamesArmLeft},
      {"arm_right",  &JointNamesArmRight},
      {"hand_left",  &JointNamesHandLeft},
      {"hand_right", &JointNamesHandRight},
      {"head",       &JointNamesHead},
      {"body",       &JointNamesBody},
    };

    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    auto it = group_map.find(grp);
    if (it != group_map.end()) {
      const auto & target_names = *it->second;
      for (size_t i = 0; i < names.size(); ++i) {
        if (std::find(target_names.begin(), target_names.end(), names[i]) != target_names.end()) {
          traj.joint_names.push_back(names[i]);
          point.positions.push_back(rads[i]);
        }
      }
    }

    if (!traj.joint_names.empty()) {
      point.time_from_start = dur;
      traj.points.push_back(point);
    }

    return traj;
  }

  void JointActionServer::load_joint_limits()
  {
    if (urdf_loaded_) {
      return;
    }
    const std::string xml = get_parameter("robot_description").as_string();
    if (xml.empty()) {
      return;
    }
    if (!parse_urdf_limits(xml)) {
      return;
    }
    urdf_loaded_ = true;
    if (urdf_timer_) {
      urdf_timer_->cancel();
    }
  }

  bool JointActionServer::parse_urdf_limits(const std::string & xml)
  {
    urdf::Model model;
    if (!model.initString(xml)) {
      return false;
    }
    for (const auto & [name, joint] : model.joints_) {
      if (joint->limits) {
        joint_limits_[name] = Limit{
          joint->limits->lower,
          joint->limits->upper,
          joint->limits->velocity,
          joint->limits->effort,
          true};
      }
    }
    return true;
  }

  std::string JointActionServer::get_tf_frame(const std::string & frame_name)
  {
    if (!get_parameter("enable_tf_prefix").as_bool()) {
      return frame_name;
    }
    std::string ns = get_namespace();
    if (!ns.empty() && ns.front() == '/') {
      ns = ns.substr(1);
    }
    return ns.empty() ? frame_name : ns + "/" + frame_name;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::JointActionServer)
