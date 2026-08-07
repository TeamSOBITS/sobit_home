#include "sobit_home_library/sobit_home_joint_action_server.hpp"

namespace sobit_home
{
JointActionServer::JointActionServer(const rclcpp::NodeOptions & options)
: Node("joint_action_server", options),
  poses_(),
  curt_joint_state_(),
  curt_joint_effort_(),
  curt_joint_velocity_(), 
  hand_left_target_joint_rad_(),    
  hand_right_target_joint_rad_(),  
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
      std::thread{[this, goal_handle]() {exe_move_joints(goal_handle);}}.detach();
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
      std::thread{[this, goal_handle]() {exe_move_to_pose(goal_handle);}}.detach();
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
      std::thread{[this, goal_handle]() {exe_move_right_hand_to_pose(goal_handle);}}.detach();
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
      std::thread{[this, goal_handle]() {exe_move_left_hand_to_pose(goal_handle);}}.detach();
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
  // Added new publisher: t.tsukada
  pub_left_hand_grasp_state_ = create_publisher<std_msgs::msg::Bool>(
    "hand_left/grasp_state", qos_profile);
  pub_right_hand_grasp_state_ = create_publisher<std_msgs::msg::Bool>(
    "hand_right/grasp_state", qos_profile);

  declare_parameter("robot_description", "");
  urdf_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() {load_joint_limits();});

  // Added timer for  grasp state monitor: t.tsukada
  grasp_monitor_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      check_grasp(true);
      check_grasp(false);});

  declare_parameter("poses", std::vector<std::string>());
  declare_parameter("right_hand_poses", std::vector<std::string>());
  declare_parameter("left_hand_poses", std::vector<std::string>());
  declare_parameter("enable_tf_prefix", false);

  load_poses_from_params();

  service_reload_poses_ = create_service<Trigger>(
      "reload_poses",
    [this](
      const std::shared_ptr<Trigger::Request> req,
      std::shared_ptr<Trigger::Response> res) {
      serve_reload_poses(req, res);
      });

  RCLCPP_INFO(get_logger(), "JointActionServer has been initialized.");
}

JointActionServer::~JointActionServer() {}

void JointActionServer::load_poses_from_params()
{
  auto try_declare = [this](const std::string & name, double default_val) {
      try {
        declare_parameter(name, default_val);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      }
    };

  const auto pose_names = get_parameter("poses").as_string_array();
  const auto right_hand_pose_names = get_parameter("right_hand_poses").as_string_array();
  const auto left_hand_pose_names = get_parameter("left_hand_poses").as_string_array();

  std::vector<PoseParams> new_poses, new_right, new_left;

  for (const auto & pose_name : pose_names) {
    for (const auto & key : {
        "arm_right_shoulder_tilt", "arm_right_upper_roll", "arm_right_upper_flex",
        "arm_right_elbow", "arm_right_lower_flex", "arm_right_wrist_tilt", "arm_right_wrist_roll",
        "hand_right_finger_l_mcp", "hand_right_finger_l_pip", "hand_right_finger_l_dip",
        "hand_right_finger_c_mcp", "hand_right_finger_c_ip", "hand_right_finger_r_pip",
        "hand_right_finger_r_dip",
        "arm_left_shoulder_tilt", "arm_left_upper_roll", "arm_left_upper_flex",
        "arm_left_elbow", "arm_left_lower_flex", "arm_left_wrist_tilt", "arm_left_wrist_roll",
        "hand_left_finger_l_mcp", "hand_left_finger_l_pip", "hand_left_finger_l_dip",
        "hand_left_finger_c_mcp", "hand_left_finger_c_ip", "hand_left_finger_r_pip",
        "hand_left_finger_r_dip",
        "body_lift", "head_pan", "head_tilt"})
    {
      try_declare(pose_name + "." + key, 0.0);
    }

    auto get = [&](const std::string & key) {
        return get_parameter(pose_name + "." + key).as_double();
      };

    PoseParams p;
    p.pose_name = pose_name;
    p.arm_right_shoulder_tilt = get("arm_right_shoulder_tilt");
    p.arm_right_upper_roll = get("arm_right_upper_roll");
    p.arm_right_upper_flex = get("arm_right_upper_flex");
    p.arm_right_elbow = get("arm_right_elbow");
    p.arm_right_lower_flex = get("arm_right_lower_flex");
    p.arm_right_wrist_tilt = get("arm_right_wrist_tilt");
    p.arm_right_wrist_roll = get("arm_right_wrist_roll");
    p.hand_right_finger_l_mcp = get("hand_right_finger_l_mcp");
    p.hand_right_finger_l_pip = get("hand_right_finger_l_pip");
    p.hand_right_finger_l_dip = get("hand_right_finger_l_dip");
    p.hand_right_finger_c_mcp = get("hand_right_finger_c_mcp");
    p.hand_right_finger_c_ip = get("hand_right_finger_c_ip");
    p.hand_right_finger_r_pip = get("hand_right_finger_r_pip");
    p.hand_right_finger_r_dip = get("hand_right_finger_r_dip");
    p.arm_left_shoulder_tilt = get("arm_left_shoulder_tilt");
    p.arm_left_upper_roll = get("arm_left_upper_roll");
    p.arm_left_upper_flex = get("arm_left_upper_flex");
    p.arm_left_elbow = get("arm_left_elbow");
    p.arm_left_lower_flex = get("arm_left_lower_flex");
    p.arm_left_wrist_tilt = get("arm_left_wrist_tilt");
    p.arm_left_wrist_roll = get("arm_left_wrist_roll");
    p.hand_left_finger_l_mcp = get("hand_left_finger_l_mcp");
    p.hand_left_finger_l_pip = get("hand_left_finger_l_pip");
    p.hand_left_finger_l_dip = get("hand_left_finger_l_dip");
    p.hand_left_finger_c_mcp = get("hand_left_finger_c_mcp");
    p.hand_left_finger_c_ip = get("hand_left_finger_c_ip");
    p.hand_left_finger_r_pip = get("hand_left_finger_r_pip");
    p.hand_left_finger_r_dip = get("hand_left_finger_r_dip");
    p.body_lift = get("body_lift");
    p.head_pan = get("head_pan");
    p.head_tilt = get("head_tilt");
    new_poses.push_back(p);
  }

  for (const auto & name : right_hand_pose_names) {
    for (const auto & key : {
        "hand_right_finger_l_mcp", "hand_right_finger_l_pip", "hand_right_finger_l_dip",
        "hand_right_finger_c_mcp", "hand_right_finger_c_ip", "hand_right_finger_r_pip",
        "hand_right_finger_r_dip"})
    {
      try_declare(name + "." + key, 0.0);
    }
    auto get = [&](const std::string & key) {return get_parameter(name + "." + key).as_double();};
    PoseParams p;
    p.pose_name = name;
    p.hand_right_finger_l_mcp = get("hand_right_finger_l_mcp");
    p.hand_right_finger_l_pip = get("hand_right_finger_l_pip");
    p.hand_right_finger_l_dip = get("hand_right_finger_l_dip");
    p.hand_right_finger_c_mcp = get("hand_right_finger_c_mcp");
    p.hand_right_finger_c_ip = get("hand_right_finger_c_ip");
    p.hand_right_finger_r_pip = get("hand_right_finger_r_pip");
    p.hand_right_finger_r_dip = get("hand_right_finger_r_dip");
    new_right.push_back(p);
  }

  for (const auto & name : left_hand_pose_names) {
    for (const auto & key : {
        "hand_left_finger_l_mcp", "hand_left_finger_l_pip", "hand_left_finger_l_dip",
        "hand_left_finger_c_mcp", "hand_left_finger_c_ip", "hand_left_finger_r_pip",
        "hand_left_finger_r_dip"})
    {
      try_declare(name + "." + key, 0.0);
    }
    auto get = [&](const std::string & key) {return get_parameter(name + "." + key).as_double();};
    PoseParams p;
    p.pose_name = name;
    p.hand_left_finger_l_mcp = get("hand_left_finger_l_mcp");
    p.hand_left_finger_l_pip = get("hand_left_finger_l_pip");
    p.hand_left_finger_l_dip = get("hand_left_finger_l_dip");
    p.hand_left_finger_c_mcp = get("hand_left_finger_c_mcp");
    p.hand_left_finger_c_ip = get("hand_left_finger_c_ip");
    p.hand_left_finger_r_pip = get("hand_left_finger_r_pip");
    p.hand_left_finger_r_dip = get("hand_left_finger_r_dip");
    new_left.push_back(p);
  }

  std::lock_guard<std::mutex> lock(poses_mutex_);
  poses_ = std::move(new_poses);
  right_hand_poses_ = std::move(new_right);
  left_hand_poses_ = std::move(new_left);
}

void JointActionServer::serve_reload_poses(
  const std::shared_ptr<Trigger::Request>/*req*/,
  std::shared_ptr<Trigger::Response> res)
{
  try {
    load_poses_from_params();
    std::string names;
    {
      std::lock_guard<std::mutex> lock(poses_mutex_);
      for (const auto & p : poses_) {
        names += " " + p.pose_name;
      }
      res->message = "Reloaded " + std::to_string(poses_.size()) + " poses, " +
        std::to_string(right_hand_poses_.size()) + " right-hand poses, " +
        std::to_string(left_hand_poses_.size()) + " left-hand poses. Whole-body poses:" + names;
    }
    res->success = true;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } catch (const std::exception & e) {
    res->success = false;
    res->message = std::string("reload_poses failed: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

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
      auto traj = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance,
        grp);
      if (!traj.joint_names.empty()) {
        try {
          pub->publish(traj);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Publish failed in %s: %s", grp.c_str(), e.what());
        }
      }
    };

  publish_group(pub_left_arm_joint_control_, "arm_left");
  publish_group(pub_right_arm_joint_control_, "arm_right");
  publish_group(pub_left_hand_joint_control_, "hand_left");
  publish_group(pub_right_hand_joint_control_, "hand_right");
  publish_group(pub_body_joint_control_, "body");
  publish_group(pub_head_joint_control_, "head");

  auto start_time = now();
  while (now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      return;
    }

    auto feedback = std::make_shared<MoveJoint::Feedback>();
    feedback->current_joint_names = goal->target_joint_names;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      for (const auto & name : goal->target_joint_names) {
        auto it = curt_joint_state_.find(name);
        feedback->current_joint_rad.push_back(it != curt_joint_state_.end() ? it->second : 0.0);
      }
    }
    goal_handle->publish_feedback(feedback);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  result->success = true;
  goal_handle->succeed(result);
}

bool JointActionServer::lookup_pose(
  const std::vector<PoseParams> & list, const std::string & name, PoseParams & out) const
{
  std::lock_guard<std::mutex> lock(poses_mutex_);
  auto it = std::find_if(list.begin(), list.end(), [&](const PoseParams & p) {
        return p.pose_name == name;
    });
  if (it == list.end()) {
    return false;
  }
  out = *it;
  return true;
}

bool JointActionServer::wait_until_done(
  const std::shared_ptr<GoalHandleMoveToPose> & goal_handle,
  const builtin_interfaces::msg::Duration & time_allowance,
  const std::shared_ptr<MoveToPose::Result> & result)
{
  auto start_time = now();
  while (now() - start_time < time_allowance) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      return false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  return true;
}

void JointActionServer::exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  PoseParams pose;
  if (!lookup_pose(poses_, goal->pose_name, pose)) {
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

  add(JointNamesArmRight,
    {pose.arm_right_shoulder_tilt, pose.arm_right_upper_roll, pose.arm_right_upper_flex,
      pose.arm_right_elbow, pose.arm_right_lower_flex, pose.arm_right_wrist_tilt,
      pose.arm_right_wrist_roll});
  add(JointNamesArmLeft,
    {pose.arm_left_shoulder_tilt, pose.arm_left_upper_roll, pose.arm_left_upper_flex,
      pose.arm_left_elbow, pose.arm_left_lower_flex, pose.arm_left_wrist_tilt,
      pose.arm_left_wrist_roll});
  add(JointNamesBody, {pose.body_lift});
  add(JointNamesHead, {pose.head_pan, pose.head_tilt});

  auto publish_group = [&](auto & pub, const std::string & grp) {
      auto traj = set_joints(names, rads, goal->time_allowance, grp);
      if (!traj.joint_names.empty()) {
        pub->publish(traj);
      }
    };

  publish_group(pub_left_arm_joint_control_, "arm_left");
  publish_group(pub_right_arm_joint_control_, "arm_right");
  publish_group(pub_body_joint_control_, "body");
  publish_group(pub_head_joint_control_, "head");

  if (!wait_until_done(goal_handle, goal->time_allowance, result)) {
    return;
  }
  result->success = true;
  goal_handle->succeed(result);
}

void JointActionServer::exe_move_hand_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
  const std::vector<PoseParams> & list,
  const std::vector<std::string> & joint_names,
  const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & pub,
  const std::string & group,
  const std::function<std::vector<double>(const PoseParams &)> & extract)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  PoseParams pose;
  if (!lookup_pose(list, goal->pose_name, pose)) {
    RCLCPP_WARN(get_logger(), "%s pose '%s' not found.", group.c_str(), goal->pose_name.c_str());
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  // storing vector of target joint radian into variable for avoid duplication of function calling extract(): t.tsukada
  const auto target_joint_rad = extract(pose);
  auto traj = set_joints(joint_names, target_joint_rad, goal->time_allowance, group);

  if (!traj.joint_names.empty()) {
      // keep final goal position for compare with current position: t.tsukada
      update_commanded_joint_position(joint_names, target_joint_rad, group == "hand_right");
    pub->publish(traj);
  }

  if (!wait_until_done(goal_handle, goal->time_allowance, result)) {
    return;
  }
  result->success = true;
  goal_handle->succeed(result);
}

void JointActionServer::exe_move_right_hand_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  exe_move_hand_to_pose(
    goal_handle, right_hand_poses_, JointNamesHandRight, pub_right_hand_joint_control_,
      "hand_right",
    [](const PoseParams & p) {
      return std::vector<double>{
        p.hand_right_finger_l_mcp, p.hand_right_finger_l_pip, p.hand_right_finger_l_dip,
        p.hand_right_finger_c_mcp, p.hand_right_finger_c_ip, p.hand_right_finger_r_pip,
        p.hand_right_finger_r_dip};
    });
}

void JointActionServer::exe_move_left_hand_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  exe_move_hand_to_pose(
    goal_handle, left_hand_poses_, JointNamesHandLeft, pub_left_hand_joint_control_, "hand_left",
    [](const PoseParams & p) {
      return std::vector<double>{
        p.hand_left_finger_l_mcp, p.hand_left_finger_l_pip, p.hand_left_finger_l_dip,
        p.hand_left_finger_c_mcp, p.hand_left_finger_c_ip, p.hand_left_finger_r_pip,
        p.hand_left_finger_r_dip};
    });
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
    goal_in_base = tf_buffer_->transform(request->target_coord, base_frame,
        tf2::durationFromSec(1.0));
    goal_in_lift = tf_buffer_->transform(request->target_coord, lift_frame,
        tf2::durationFromSec(1.0));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "TF transform failed: %s", ex.what());
    response->success = false;
    response->message = "TF error: " + std::string(ex.what());
    return;
  }

    // Height gate: base_footprint is on the floor, so goal_in_base.z is the
    // target height. Outside the graspable band -> unreachable by any move.
  const double target_height = goal_in_base.transform.translation.z;
  if (target_height > MAX_GRASP_HEIGHT || target_height < MIN_GRASP_HEIGHT) {
    response->success = false;
    response->message = "Target out of range: height " +
      std::to_string(target_height) + " m is outside graspable band [" +
      std::to_string(MIN_GRASP_HEIGHT) + ", " + std::to_string(MAX_GRASP_HEIGHT) + "] m.";
    return;
  }

  auto rads = kinematics_->inverse_kinematics(goal_in_lift, is_right);

    // move_pose = base/lift reposition (zero if reachable in place). When not
    // reachable in place, post_move_rads holds the arm config after the move.
  std::vector<double> post_move_rads;
  response->move_pose =
    kinematics_->forward_kinematics(rads, goal_in_base, goal_in_lift, is_right, &post_move_rads);

    // success == IK solvable: in place (rads) or after move (post_move_rads);
    // false only when no arm config reaches it even after the optimal move.
  const bool in_place = !rads.empty();
  const std::vector<double> & joints = in_place ? rads : post_move_rads;

  if (joints.empty()) {
    response->success = false;
    response->message = "Target unreachable: no arm solution within workspace.";
    return;
  }

  response->target_joint_names = is_right ? JointNamesArmRight : JointNamesArmLeft;
  response->target_joint_rad = joints;
  response->success = true;

  if (in_place) {
    response->message = "Success: target reachable in place.";
  } else {
    const double dx = response->move_pose.position.x;
    const double dz = response->move_pose.position.z;
    std::string msg = "Success: reachable after move:";
    if (std::abs(dz) > 0.001) {
      msg += " adjust lift " + std::to_string(std::abs(dz)) + " m " +
        (dz > 0 ? "up" : "down") + ";";
    }
    if (std::abs(dx) > 0.001) {
      msg += " drive base " + std::to_string(std::abs(dx)) + " m " +
        (dx > 0 ? "forward" : "backward") + ";";
    }
    response->message = msg;
  }
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
    goal_in_base = tf_buffer_->lookupTransform(base_frame, request->target_frame,
        tf2::TimePointZero, tf2::durationFromSec(1.0));
    goal_in_lift = tf_buffer_->lookupTransform(lift_frame, request->target_frame,
        tf2::TimePointZero, tf2::durationFromSec(1.0));
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

    // Height gate: base_footprint is on the floor, so goal_in_base.z is the
    // target height. Outside the graspable band -> unreachable by any move.
  const double target_height = goal_in_base.transform.translation.z;
  if (target_height > MAX_GRASP_HEIGHT || target_height < MIN_GRASP_HEIGHT) {
    response->success = false;
    response->message = "Target out of range: height " +
      std::to_string(target_height) + " m is outside graspable band [" +
      std::to_string(MIN_GRASP_HEIGHT) + ", " + std::to_string(MAX_GRASP_HEIGHT) + "] m.";
    return;
  }

  const auto rads = kinematics_->inverse_kinematics(goal_in_lift, is_right);

    // move_pose = base/lift reposition (zero if reachable in place). When not
    // reachable in place, post_move_rads holds the arm config after the move.
  std::vector<double> post_move_rads;
  response->move_pose =
    kinematics_->forward_kinematics(rads, goal_in_base, goal_in_lift, is_right, &post_move_rads);

    // success == IK solvable: in place (rads) or after move (post_move_rads);
    // false only when no arm config reaches it even after the optimal move.
  const bool in_place = !rads.empty();
  const std::vector<double> & joints = in_place ? rads : post_move_rads;

  if (joints.empty()) {
    response->success = false;
    response->message = "Target unreachable: no arm solution within workspace.";
    return;
  }

  response->target_joint_names = is_right ? JointNamesArmRight : JointNamesArmLeft;
  response->target_joint_rad = joints;
  response->success = true;

  if (in_place) {
    response->message = "Success: TF target reachable in place.";
  } else {
    const double dx = response->move_pose.position.x;
    const double dz = response->move_pose.position.z;
    std::string msg = "Success: reachable after move:";
    if (std::abs(dz) > 0.001) {
      msg += " adjust lift " + std::to_string(std::abs(dz)) + " m " +
        (dz > 0 ? "up" : "down") + ";";
    }
    if (std::abs(dx) > 0.001) {
      msg += " drive base " + std::to_string(std::abs(dx)) + " m " +
        (dx > 0 ? "forward" : "backward") + ";";
    }
    response->message = msg;
  }
}

void JointActionServer::get_head_to_coord(
  const std::shared_ptr<GetHandToTargetCoord::Request> request,
  std::shared_ptr<GetHandToTargetCoord::Response> response)
{
  geometry_msgs::msg::TransformStamped goal_head;
  try {
    goal_head = tf_buffer_->transform(request->target_coord, get_tf_frame("head_base_link"),
        tf2::durationFromSec(1.0));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
    response->success = false;
    return;
  }

  response->target_joint_names = JointNamesHead;
  response->target_joint_rad = kinematics_->look_at(goal_head);
  response->success = true;
}

void JointActionServer::get_head_to_tf(
  const std::shared_ptr<GetHandToTargetTF::Request> request,
  std::shared_ptr<GetHandToTargetTF::Response> response)
{
  geometry_msgs::msg::TransformStamped goal_head;
  try {
    goal_head = tf_buffer_->lookupTransform(get_tf_frame("head_base_link"), request->target_frame,
        tf2::TimePointZero);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
    response->success = false;
    return;
  }

  response->target_joint_names = JointNamesHead;
  response->target_joint_rad = kinematics_->look_at(goal_head);
  response->success = true;
}

void JointActionServer::serve_get_finger_angle(
  const std::shared_ptr<GetFingerAngle::Request> request,
  std::shared_ptr<GetFingerAngle::Response> response)
{
  response->target_joint_names = request->is_right ? JointNamesHandRight : JointNamesHandLeft;
  if (request->grasp_form == 0) {
    response->opened_target_joint_rad = {-1.65, 1.571, -1.0, -1.571, 1.0, -1.571, 1.0};
    response->closed_target_joint_rad = (request->grasp_mode == "pick") ?
      std::vector<double>(7, 0.0) :
      std::vector<double>{-1.65, -0.1, -0.8, -0.1, 1.0, 0.1, 0.8};
  }
  response->success = true;
}

void JointActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  // position/velocity/effort may each be shorter than name (or empty) per sensor_msgs/JointState
  if (msg->position.size() < msg->name.size() || msg->effort.size() < msg->name.size() ||
      msg->velocity.size() < msg->name.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "JointState has %zu names but %zu positions, %zu velocities, %zu efforts; ignoring the remainder",
      msg->name.size(), msg->position.size(), msg->velocity.size(), msg->effort.size());
  }
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->position.size()) curt_joint_state_[msg->name[i]] = msg->position[i];
    // Added effort and velosity parameter to judge open/close of hand: t.tsukada
    if (i < msg->effort.size()) curt_joint_effort_[msg->name[i]] = msg->effort[i];
    if (i < msg->velocity.size()) curt_joint_velocity_[msg->name[i]] = msg->velocity[i];
  }
}

//Keep commanded radian of hand joints
void JointActionServer::update_commanded_joint_position(
    const std::vector<std::string>& names,
    const std::vector<double>& rads,
    bool is_right)
{
    std::map<std::string, double>& target_map =
        is_right ? hand_right_target_joint_rad_
                : hand_left_target_joint_rad_;

    for(size_t i = 0; i < names.size(); i++)
    {
        target_map[names[i]] = rads[i];
    }
}

// check grasp result
void JointActionServer::check_grasp(bool is_right)
{
  const auto & target_map =
    is_right ? hand_right_target_joint_rad_
             : hand_left_target_joint_rad_;

  if (target_map.empty()) {
    std_msgs::msg::Bool msg;
    msg.data = false;

    (is_right ? pub_right_hand_grasp_state_
              : pub_left_hand_grasp_state_)->publish(msg);
    return;
  }

  constexpr double angle_th = 0.025;
  constexpr double vel_th = 0.035;
  
  constexpr double current_th_mA = 20.0;   
  constexpr double current_scale = 2.69;   

  const std::vector<std::vector<std::string>> fingers =
    is_right ?
    std::vector<std::vector<std::string>> {
      {"hand_right_finger_l_mcp_joint",
       "hand_right_finger_l_pip_joint",
       "hand_right_finger_l_dip_joint"},
      {"hand_right_finger_c_mcp_joint",
       "hand_right_finger_c_ip_joint"},
      {"hand_right_finger_r_pip_joint",
       "hand_right_finger_r_dip_joint"}
    } :
    std::vector<std::vector<std::string>> {
      {"hand_left_finger_l_mcp_joint",
       "hand_left_finger_l_pip_joint",
       "hand_left_finger_l_dip_joint"},
      {"hand_left_finger_c_mcp_joint",
       "hand_left_finger_c_ip_joint"},
      {"hand_left_finger_r_pip_joint",
       "hand_left_finger_r_dip_joint"}
    };

  int valid_fingers = 0;

  for (const auto & finger : fingers)
  {
    int blocked_joint = 0;

    for (const auto & joint : finger)
    {
      auto it_q = curt_joint_state_.find(joint);
      auto it_t = target_map.find(joint);
      auto it_v = curt_joint_velocity_.find(joint);
      auto it_e = curt_joint_effort_.find(joint);

      if (it_q == curt_joint_state_.end() ||
          it_t == target_map.end() ||
          it_v == curt_joint_velocity_.end() ||
          it_e == curt_joint_effort_.end()) {
        continue;
      }

      const double q_err = std::abs(it_q->second - it_t->second);
      const double v = std::abs(it_v->second);

      const double e_mA_raw = std::abs(it_e->second);
      const double e_mA = e_mA_raw /current_scale; 

      const bool position_blocked = (q_err > angle_th);
      const bool nearly_stopped = (v < vel_th);
      const bool current_increased = (e_mA > current_th_mA);

      if (position_blocked && nearly_stopped && current_increased) {
        blocked_joint++;
      }
    }

    const bool finger_grasped = (blocked_joint >= 1);

    if (finger_grasped) {
      valid_fingers++;
    }
  }

  bool is_grasped = (valid_fingers >= 2);

  std_msgs::msg::Bool msg;
  msg.data = is_grasped;

  (is_right ? pub_right_hand_grasp_state_
            : pub_left_hand_grasp_state_)->publish(msg);
}

trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(
  const std::vector<std::string> & names,
  const std::vector<double> & rads,
  const builtin_interfaces::msg::Duration & dur,
  const std::string & grp)
{
  static const std::unordered_map<std::string, const std::vector<std::string> *> group_map = {
    {"arm_left", &JointNamesArmLeft},
    {"arm_right", &JointNamesArmRight},
    {"hand_left", &JointNamesHandLeft},
    {"hand_right", &JointNamesHandRight},
    {"head", &JointNamesHead},
    {"body", &JointNamesBody},
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
