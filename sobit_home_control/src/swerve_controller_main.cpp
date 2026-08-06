#include "sobit_home_control/swerve_controller_main.hpp"

namespace sobit_home
{

SwerveController::SwerveController(const rclcpp::NodeOptions & options)
: Node("swerve_controller", options)
{
  declare_parameter("robot_base_frame", "base_footprint");
  declare_parameter("twist_topic", "cmd_vel");
  declare_parameter("position_controller_name", "wheel_steer_position_controller");
  declare_parameter("velocity_controller_name", "wheel_drive_velocity_controller");
  declare_parameter("cycle_fequency", 10);
  declare_parameter("steering_joints", std::vector<std::string>{
    "wheel_steer_f_l_joint", "wheel_steer_f_r_joint",
    "wheel_steer_b_l_joint", "wheel_steer_b_r_joint"});
  declare_parameter("drive_joints", std::vector<std::string>{
    "wheel_drive_f_l_joint", "wheel_drive_f_r_joint",
    "wheel_drive_b_l_joint", "wheel_drive_b_r_joint"});
  declare_parameter("mobile_base.wheel_radius",      0.075);
  declare_parameter("mobile_base.wheel_x_distance",  0.35355339);
  declare_parameter("mobile_base.wheel_y_distance",  0.35355339);
  declare_parameter("mobile_base.steer_max_vel",     10.0);
  declare_parameter("mobile_base.drive_max_vel",     10.0);
  // NOTE: the config nests this under mobile_base; the old top-level
  // "driving_status_threshold" was never populated from swerve_config.yaml,
  // so the robot silently ran with the 0.26 rad default.
  declare_parameter("mobile_base.driving_status_threshold", 0.05);
  declare_parameter("mobile_base.steer_settle_vel",         0.5);

  steering_joints_names = get_parameter("steering_joints").as_string_array();
  drive_joints_names    = get_parameter("drive_joints").as_string_array();

  if ((steering_joints_names.size() != 4) || (drive_joints_names.size() != 4)) return;

  CYCLE_FEQUENCY           = get_parameter("cycle_fequency").as_int();
  STEER_MAX_VEL            = get_parameter("mobile_base.steer_max_vel").as_double();
  DRIVING_STATUS_THRESHOLD = get_parameter("mobile_base.driving_status_threshold").as_double();

  // Initialize the control and odometry classes
  sobit_home_control_ = std::make_unique<SobitHomeControl>(this);
  sobit_home_odometry_ = std::make_unique<SobitHomeOdometry>(this);

  // Configure the QoS profile
  rclcpp::QoS qos_profile(1);
  // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  sub_vel_ = create_subscription<geometry_msgs::msg::Twist>(
    get_parameter("twist_topic").as_string(), qos_profile,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      sobit_home_control_->twist_callback(msg);
    });
  sub_joint_info_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", qos_profile,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_callback(msg); });

  pub_odometry_    = create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);
  pub_steer_joint_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    get_parameter("position_controller_name").as_string() + "/commands", qos_profile);
  pub_wheel_joint_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    get_parameter("velocity_controller_name").as_string() + "/commands", qos_profile);

  joints_pos.clear();
  while (joints_pos.empty()) { rclcpp::spin_some(get_node_base_interface()); }

  for (size_t i=0; i < drive_joints_names.size(); i++) {
    sobit_home_control_->current_steer_pos[i] = sobit_home_odometry_->current_steer_pos[i] = sobit_home_control_->goal_steer_pos[i] = joints_pos[steering_joints_names[i]];
    sobit_home_odometry_->prev_steer_pos[i] = joints_pos[steering_joints_names[i]];
    sobit_home_control_->goal_drive_vel[i] = 0.0;
    sobit_home_odometry_->prev_drive_pos[i] = sobit_home_odometry_->current_drive_pos[i] = joints_pos[drive_joints_names[i]];
  }

  prev_cycle_time_ = get_clock()->now();

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / CYCLE_FEQUENCY)),
    [this]() { control_callback(); });

  sobit_home_odometry_->odom_.header.stamp    = get_clock()->now();
  sobit_home_odometry_->odom_.header.frame_id = "odom";
  sobit_home_odometry_->odom_.child_frame_id  = get_parameter("robot_base_frame").as_string();

  // Start up sound
  play_sound(true);

  RCLCPP_INFO(this->get_logger(), "SOBIT HOME Wheel Main initialized.");
}

SwerveController::~SwerveController()
{
  RCLCPP_INFO(this->get_logger(), "SOBIT HOME Wheel Main destroyed.");
  // Shut down sound
  play_sound(false);
}

void SwerveController::joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_info)
{
  for (size_t i = 0; i < joint_info->name.size(); ++i) 
    joints_pos[joint_info->name[i]] = joint_info->position[i];
}

// Play sound func
void SwerveController::play_sound(bool is_startup)
{
  // Get sound file path
  std::string pack_path = ament_index_cpp::get_package_share_directory("sobit_home_control");
  std::string sound_file = is_startup ? "start_up" : "shut_down";
  std::string sound_path = pack_path + "/mp3/" + sound_file + ".mp3";

  // // Play Sound
  std::system(("mpg321 --quiet " + sound_path + " &").c_str());
}


// Control wheel
void SwerveController::control_callback()
{
  // Wait until joint_states topic is populated (needed for steering and odometry calculations)
  if (joints_pos.empty()) return;

  // Measured cycle time — the wall timer and the joint_states publisher are
  // unsynchronized, so the nominal period is up to one joint_states period off.
  rclcpp::Time now = get_clock()->now();
  double dt = (now - prev_cycle_time_).seconds();
  prev_cycle_time_ = now;
  if (dt <= 1e-6 || dt > 1.0) dt = 1.0 / CYCLE_FEQUENCY;

  // Update current steer positions
  for (size_t i=0; i < steering_joints_names.size(); i++) {
    sobit_home_control_->current_steer_pos[i] = sobit_home_odometry_->current_steer_pos[i] = joints_pos[steering_joints_names[i]];
    sobit_home_odometry_->current_drive_pos[i] = joints_pos[drive_joints_names[i]];
  }

  // Determine steering state:
  //   1 = steers at goal → publish goal drive velocities
  //   0 = steers adjusting within threshold → publish 0 (drives stop)
  //  -1 = steers far from goal → publish 0 (drives stop)
  //
  // Previously, state==0 did NOT publish, leaving the velocity controller at
  // its last setpoint while the steer was at an intermediate angle.  This
  // caused odometry to integrate a wrong heading and produced localization
  // drift during every mode transition (swivel/rotation).  Now we always
  // publish an explicit command so the controller is never in open-loop.
  //
  // NOTE: the gate is deliberately position-error-only and moderate. Teleop
  // nudges the steering goals continuously, so a strict gate (tight error +
  // steer-speed condition) chops drive power several times per second — jerky
  // base. Odometry accuracy during transitions does NOT depend on this gate:
  // update_odom() excludes still-steering wheels per wheel (steer_settle_vel).
  int steering_state = 1;
  wheel_joint_pos.data.clear();
  for (size_t i=0; i < steering_joints_names.size(); i++) {
    wheel_joint_pos.data.push_back(sobit_home_control_->goal_steer_pos[i]);
    if (steering_state != -1) {
      double err = fabs(sobit_home_control_->goal_steer_pos[i] - sobit_home_control_->current_steer_pos[i]);
      if (err > (DRIVING_STATUS_THRESHOLD + STEER_MAX_VEL / CYCLE_FEQUENCY))
        steering_state = -1;
      else if (err > DRIVING_STATUS_THRESHOLD)
        steering_state = 0;
    }
  }

  pub_steer_joint_->publish(wheel_joint_pos);

  // Always publish drive command — never leave the velocity controller without
  // an authoritative setpoint.  Drives run only when steer is settled (state 1).
  wheel_joint_vel.data.clear();
  for (size_t i=0; i < drive_joints_names.size(); i++)
    wheel_joint_vel.data.push_back((steering_state == 1) ? sobit_home_control_->goal_drive_vel[i] : 0.0);
  pub_wheel_joint_->publish(wheel_joint_vel);

  // Integrate only when steering has settled (mid-transition angles corrupt the
  // odom solve). Otherwise hold pose + zero twist (drives are 0, base isn't moving).
  // Always publish odom + TF so Nav2 never sees a transform gap.
  if (steering_state == 1) {
    sobit_home_odometry_->update_odom(dt);
  } else {
    sobit_home_odometry_->odom_.twist.twist = geometry_msgs::msg::Twist();
    sobit_home_odometry_->odom_.header.stamp = get_clock()->now();
  }
  sobit_home_odometry_->pose_broadcaster();
  pub_odometry_->publish(sobit_home_odometry_->odom_);

  // Sync every cycle so coast during settling isn't integrated as a jump on resume.
  for (int i=0; i<4; i++) {
    sobit_home_odometry_->prev_drive_pos[i] = sobit_home_odometry_->current_drive_pos[i];
    sobit_home_odometry_->prev_steer_pos[i] = sobit_home_odometry_->current_steer_pos[i];
  }
}

} // namespace sobit_home