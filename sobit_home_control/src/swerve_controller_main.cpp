#include "sobit_home_control/swerve_controller_main.hpp"

namespace sobit_home{

SwerveController::SwerveController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("swerve_controller", options)
{
  // declare parameters
  this->declare_parameter("robot_base_frame", "base_footprint");
  this->declare_parameter("twist_topic", "cmd_vel");
  this->declare_parameter("position_controller_name", "wheel_steer_position_controller");
  this->declare_parameter("velocity_controller_name", "wheel_drive_velocity_controller");
  this->declare_parameter("cycle_fequency", 10);
  this->declare_parameter("steering_joints", std::vector<std::string>({"wheel_steer_f_l_joint", "wheel_steer_f_r_joint", "wheel_steer_b_l_joint", "wheel_steer_b_r_joint"}));
  this->declare_parameter("drive_joints",    std::vector<std::string>({"wheel_drive_f_l_joint", "wheel_drive_f_r_joint", "wheel_drive_b_l_joint", "wheel_drive_b_r_joint"}));
  this->declare_parameter("mobile_base.wheel_radius", 0.075);
  // this->declare_parameter("mobile_base.wheel_width" , 0.035);
  this->declare_parameter("mobile_base.wheel_x_distance", 0.35355339);
  this->declare_parameter("mobile_base.wheel_y_distance", 0.35355339);
  this->declare_parameter("mobile_base.steer_max_vel", 10.0);
  // this->declare_parameter("mobile_base.steer_min_acc", 0.5);
  // this->declare_parameter("mobile_base.steer_max_acc", 1.0);
  this->declare_parameter("mobile_base.drive_max_vel", 10.0);
  // this->declare_parameter("mobile_base.drive_min_acc", 0.5);
  // this->declare_parameter("mobile_base.drive_max_acc", 1.0);
  this->declare_parameter("driving_status_threshold", 0.26);


  steering_joints_names = this->get_parameter("steering_joints").as_string_array();
  drive_joints_names    = this->get_parameter("drive_joints").as_string_array();

  if ((steering_joints_names.size() != 4) || (drive_joints_names.size() != 4)) return;

  CYCLE_FEQUENCY = this->get_parameter("cycle_fequency").as_int();
  STEER_MAX_VEL = this->get_parameter("mobile_base.steer_max_vel").as_double();
  DRIVING_STATUS_THRESHOLD = this->get_parameter("driving_status_threshold").as_double();

  // Initialize the control and odometry classes
  sobit_home_control_ = std::make_unique<SobitHomeControl>(this);
  sobit_home_odometry_ = std::make_unique<SobitHomeOdometry>(this);

  // Configure the QoS profile
  rclcpp::QoS qos_profile(1);
  // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  this->sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      this->get_parameter("twist_topic").as_string(), qos_profile, std::bind(&SobitHomeControl::twist_callback, sobit_home_control_.get(), std::placeholders::_1));
  this->sub_joint_info_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&SwerveController::joint_callback, this, std::placeholders::_1));

  this->pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos_profile);
  this->pub_steer_joint_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      this->get_parameter("position_controller_name").as_string() + "/commands", qos_profile);
  this->pub_wheel_joint_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      this->get_parameter("velocity_controller_name").as_string() + "/commands", qos_profile);

  // Set the initial position of the wheel
  joints_pos.clear();
  while (joints_pos.empty()) rclcpp::spin_some(this->get_node_base_interface());

  for (size_t i=0; i < drive_joints_names.size(); i++) {
    sobit_home_control_->current_steer_pos[i] = sobit_home_odometry_->current_steer_pos[i] = sobit_home_control_->goal_steer_pos[i] = joints_pos[steering_joints_names[i]];
    sobit_home_control_->goal_drive_vel[i] = 0.0;
    sobit_home_odometry_->prev_drive_pos[i] = sobit_home_odometry_->current_drive_pos[i] = joints_pos[drive_joints_names[i]];
  }

  // create looped function of 50hz
  this->control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((int)(1000. / CYCLE_FEQUENCY)),
      std::bind(&SwerveController::control_callback, this));

  // Get the robot namespace
  std::string robot_name = (std::strcmp(this->get_namespace(), "/") != 0)
                          ? std::string(this->get_namespace()).substr(1) + "/"
                          : "";

  std::string base_frame_name_ = this->get_parameter("robot_base_frame").as_string();
  
  // Initilize Odometry
  sobit_home_odometry_->odom_.header.stamp    = this->get_clock()->now();
  sobit_home_odometry_->odom_.header.frame_id = robot_name + "odom";
  sobit_home_odometry_->odom_.child_frame_id  = robot_name + this->get_parameter("robot_base_frame").as_string();

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

  // Update current steer positions
  for (size_t i=0; i < steering_joints_names.size(); i++) {
    sobit_home_control_->current_steer_pos[i] = sobit_home_odometry_->current_steer_pos[i] = joints_pos[steering_joints_names[i]];
    sobit_home_odometry_->current_drive_pos[i] = joints_pos[drive_joints_names[i]];
  }

  // Publish Float64MultiArray of Steering [rad/s]
  int steering_state = 1; // -1: STOP, 0: CONTINUE(non-publish), 1: Publish
  wheel_joint_pos.data.clear();
  for (size_t i=0; i < steering_joints_names.size(); i++) {
    wheel_joint_pos.data.push_back(sobit_home_control_->goal_steer_pos[i]);
    if (steering_state != -1) {
      if (fabs(sobit_home_control_->goal_steer_pos[i] - sobit_home_control_->current_steer_pos[i]) > (DRIVING_STATUS_THRESHOLD + STEER_MAX_VEL/CYCLE_FEQUENCY)) {
        steering_state = -1;
      } else if (fabs(sobit_home_control_->goal_steer_pos[i] - sobit_home_control_->current_steer_pos[i]) > DRIVING_STATUS_THRESHOLD) {
        steering_state = 0;
      }
    }
  }

  pub_steer_joint_->publish(wheel_joint_pos);

  // Publish Float64MultiArray of Driving [rad/s]
  if ((steering_state == 1) || (steering_state == -1)) {
    wheel_joint_vel.data.clear();

    for (size_t i=0; i < drive_joints_names.size(); i++) 
      wheel_joint_vel.data.push_back((steering_state == 1) ? sobit_home_control_->goal_drive_vel[i] : 0.0);

    pub_wheel_joint_->publish(wheel_joint_vel);
  }

  // Calculate Odometry
  sobit_home_odometry_->update_odom();

  // Publish Odometry
  sobit_home_odometry_->pose_broadcaster();
  pub_odometry_->publish(sobit_home_odometry_->odom_);

  // Update Previous data
  for (int i=0; i<4; i++)
    sobit_home_odometry_->prev_drive_pos[i] = sobit_home_odometry_->current_drive_pos[i];
}

} // namespace sobit_home