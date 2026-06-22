#include "sobit_home_library/sobit_home_wheel_action_server.hpp"

namespace sobit_home
{

WheelActionServer::WheelActionServer(const rclcpp::NodeOptions & options)
: Node("wheel_action_server", options)
{
  rclcpp::QoS qos_profile(1);

  declare_parameter("wheel_linear_kp", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("wheel_linear_ki", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("wheel_linear_kd", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("wheel_rotate_kp", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("wheel_rotate_ki", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("wheel_rotate_kd", rclcpp::PARAMETER_DOUBLE);

  wheel_linear_kp_ = get_parameter("wheel_linear_kp").as_double();
  wheel_linear_ki_ = get_parameter("wheel_linear_ki").as_double();
  wheel_linear_kd_ = get_parameter("wheel_linear_kd").as_double();
  wheel_rotate_kp_ = get_parameter("wheel_rotate_kp").as_double();
  wheel_rotate_ki_ = get_parameter("wheel_rotate_ki").as_double();
  wheel_rotate_kd_ = get_parameter("wheel_rotate_kd").as_double();

  // Arrival tolerances (m / rad) — defaulted so the node runs standalone.
  wheel_linear_arrival_tol_ = declare_parameter<double>("wheel_linear_arrival_tol", 0.02);
  wheel_rotate_arrival_tol_ = declare_parameter<double>("wheel_rotate_arrival_tol", 0.02);

  RCLCPP_INFO(get_logger(), "Wheel Linear PID parameters:");
  RCLCPP_INFO(get_logger(), "  Kp: %f", wheel_linear_kp_);
  RCLCPP_INFO(get_logger(), "  Ki: %f", wheel_linear_ki_);
  RCLCPP_INFO(get_logger(), "  Kd: %f", wheel_linear_kd_);
  RCLCPP_INFO(get_logger(), "Wheel Rotate PID parameters:");
  RCLCPP_INFO(get_logger(), "  Kp: %f", wheel_rotate_kp_);
  RCLCPP_INFO(get_logger(), "  Ki: %f", wheel_rotate_ki_);
  RCLCPP_INFO(get_logger(), "  Kd: %f", wheel_rotate_kd_);

  action_server_move_wheel_linear_ = rclcpp_action::create_server<MoveWheelLinear>(
      this, "move_wheel_linear",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveWheelLinear::Goal>) {
      RCLCPP_INFO(get_logger(), "Received move_wheel_linear goal request");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
    [this](const std::shared_ptr<GoalHandleMoveWheelLinear>) {
      RCLCPP_INFO(get_logger(), "Received move_wheel_linear cancel request");
      return rclcpp_action::CancelResponse::ACCEPT;
      },
    [this](const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle) {
      std::thread{[this, goal_handle]() {exe_move_wheel_linear(goal_handle);}}.detach();
      });

  action_server_move_wheel_rotate_ = rclcpp_action::create_server<MoveWheelRotate>(
      this, "move_wheel_rotate",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveWheelRotate::Goal>) {
      RCLCPP_INFO(get_logger(), "Received move_wheel_rotate goal request");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
    [this](const std::shared_ptr<GoalHandleMoveWheelRotate>) {
      RCLCPP_INFO(get_logger(), "Received move_wheel_rotate cancel request");
      return rclcpp_action::CancelResponse::ACCEPT;
      },
    [this](const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle) {
      std::thread{[this, goal_handle]() {exe_move_wheel_rotate(goal_handle);}}.detach();
      });

  pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos_profile,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {odom_callback(msg);});

  // Allow live tuning of PID gains + arrival tolerances via `ros2 param set`.
  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return on_param_update(params);
    });

  RCLCPP_INFO(get_logger(), "WheelActionServer has been initialized.");
}

rcl_interfaces::msg::SetParametersResult WheelActionServer::on_param_update(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if      (n == "wheel_linear_kp")          wheel_linear_kp_          = p.as_double();
    else if (n == "wheel_linear_ki")          wheel_linear_ki_          = p.as_double();
    else if (n == "wheel_linear_kd")          wheel_linear_kd_          = p.as_double();
    else if (n == "wheel_rotate_kp")          wheel_rotate_kp_          = p.as_double();
    else if (n == "wheel_rotate_ki")          wheel_rotate_ki_          = p.as_double();
    else if (n == "wheel_rotate_kd")          wheel_rotate_kd_          = p.as_double();
    else if (n == "wheel_linear_arrival_tol") wheel_linear_arrival_tol_ = p.as_double();
    else if (n == "wheel_rotate_arrival_tol") wheel_rotate_arrival_tol_ = p.as_double();
  }
  return result;
}

WheelActionServer::~WheelActionServer()
{
  action_server_move_wheel_linear_.reset();
  action_server_move_wheel_rotate_.reset();
  pub_cmd_vel_.reset();
  sub_odom_.reset();
  RCLCPP_INFO(get_logger(), "WheelActionServer has been terminated.");
}

void WheelActionServer::exe_move_wheel_linear(
  const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing move_wheel_linear goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelLinear::Result>();

  if (goal->target_point.z != 0.0) {
    RCLCPP_ERROR(get_logger(), "Invalid target point: (%f, %f, %f)",
        goal->target_point.x, goal->target_point.y, goal->target_point.z);
    result->success = false;
    result->message = "[FAIL] Invalid target point";
    goal_handle->abort(result);
    return;
  }

  geometry_msgs::msg::Twist zero_vel;
  const double goal_dist = std::sqrt(
      std::pow(goal->target_point.x, 2) + std::pow(goal->target_point.y, 2));
  double curt_dist = 0.0;

  double prev_error = goal_dist;
  double error_integral = 0.0;
  const double MAX_LINEAR_VEL = 0.4;
  const double MAX_INTEGRAL = 0.5;
  const double BRAKE_DIST = 0.20;
  // Arrival tolerance (param wheel_linear_arrival_tol): the PD braking (vel→0 as
  // error→0) makes curt_dist approach goal_dist ASYMPTOTICALLY, so
  // `curt_dist < goal_dist` is effectively never false → the loop spins at ~0
  // speed until the client times out even though the base arrived. Exit once
  // within tol so the goal succeeds promptly.
  const double ARRIVAL_TOL = wheel_linear_arrival_tol_;

  auto start_time = now();
  auto prev_time = start_time;
  init_odom_ = curt_odom_;
  rclcpp::Rate loop_rate(10);

  while (goal_dist - curt_dist > ARRIVAL_TOL) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "move_wheel_linear goal canceled");
      pub_cmd_vel_->publish(zero_vel);
      result->success = false;
      result->message = "[FAIL] Goal has been canceled";
      result->total_elapsed_time.sec = (now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (now() - start_time).nanoseconds() %
        static_cast<int64_t>(1e9);
      goal_handle->canceled(result);
      return;
    }

    auto curt_time = now();
    double dt = (curt_time - prev_time).nanoseconds() / 1e9;
    if (dt <= 0.0) {dt = 0.1;}
    prev_time = curt_time;

    const double error = goal_dist - curt_dist;
    error_integral = std::clamp(error_integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    const double error_derivative = (error - prev_error) / dt;
    prev_error = error;

    double vel_linear = wheel_linear_kp_ * error +
      wheel_linear_ki_ * error_integral +
      wheel_linear_kd_ * error_derivative;
    vel_linear = std::clamp(vel_linear, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);

    if (error > 0.0 && error < BRAKE_DIST) {
      vel_linear *= error / BRAKE_DIST;
    }

    const double heading = std::atan2(goal->target_point.y, goal->target_point.x);
    geometry_msgs::msg::Twist out_vel;
    out_vel.linear.x = vel_linear * std::cos(heading);
    out_vel.linear.y = vel_linear * std::sin(heading);
    pub_cmd_vel_->publish(out_vel);

    curt_dist = std::sqrt(
        std::pow(curt_odom_.pose.pose.position.x - init_odom_.pose.pose.position.x, 2) +
        std::pow(curt_odom_.pose.pose.position.y - init_odom_.pose.pose.position.y, 2));

    auto feedback = std::make_shared<MoveWheelLinear::Feedback>();
    feedback->current_point.x = curt_dist * std::cos(heading);
    feedback->current_point.y = curt_dist * std::sin(heading);
    feedback->move_time.sec = (now() - start_time).seconds();
    feedback->move_time.nanosec = (now() - start_time).nanoseconds() % static_cast<int64_t>(1e9);
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  pub_cmd_vel_->publish(zero_vel);
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (now() - start_time).nanoseconds() %
    static_cast<int64_t>(1e9);
  goal_handle->succeed(result);
}

void WheelActionServer::exe_move_wheel_rotate(
  const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing move_wheel_rotate goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelRotate::Result>();

  init_odom_ = curt_odom_;
  const double init_real_angle = get_euler_from_quat(init_odom_.pose.pose.orientation).z;
  double prev_real_angle = init_real_angle;
  double moved_angle = 0.0;
  const double goal_angle = std::abs(goal->target_yaw);

  double prev_error_rot = goal_angle;
  double error_integral_rot = 0.0;
  const double MAX_ANGULAR_VEL = 0.7;
  const double MAX_INTEGRAL_ROT = 0.5;
  const double BRAKE_ANGLE = 0.35;
  const double ARRIVAL_TOL_ROT = wheel_rotate_arrival_tol_;  // rad — exit before asymptotic PD stall

  auto start_time = now();
  auto prev_time_rot = start_time;
  rclcpp::Rate loop_rate(10);
  geometry_msgs::msg::Twist zero_vel;

  while (goal_angle - moved_angle > ARRIVAL_TOL_ROT) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "move_wheel_rotate goal canceled");
      pub_cmd_vel_->publish(zero_vel);
      result->success = false;
      result->message = "[FAIL] Goal has been canceled";
      result->total_elapsed_time.sec = (now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (now() - start_time).nanoseconds() %
        static_cast<int64_t>(1e9);
      goal_handle->canceled(result);
      return;
    }

    auto curt_time = now();
    double dt_rot = (curt_time - prev_time_rot).nanoseconds() / 1e9;
    if (dt_rot <= 0.0) {dt_rot = 0.1;}
    prev_time_rot = curt_time;

    const double error_rot = goal_angle - moved_angle;
    error_integral_rot = std::clamp(
        error_integral_rot + error_rot * dt_rot, -MAX_INTEGRAL_ROT, MAX_INTEGRAL_ROT);
    const double error_derivative_rot = (error_rot - prev_error_rot) / dt_rot;
    prev_error_rot = error_rot;

    double vel_angular = wheel_rotate_kp_ * error_rot +
      wheel_rotate_ki_ * error_integral_rot +
      wheel_rotate_kd_ * error_derivative_rot;

    if (error_rot > 0.0 && error_rot < BRAKE_ANGLE) {
      vel_angular *= error_rot / BRAKE_ANGLE;
    }

    vel_angular = std::clamp(std::abs(vel_angular), 0.0, MAX_ANGULAR_VEL) *
      (goal->target_yaw > 0 ? 1.0 : -1.0);

    geometry_msgs::msg::Twist out_vel;
    out_vel.angular.z = vel_angular;
    pub_cmd_vel_->publish(out_vel);

    const double curt_real_angle = get_euler_from_quat(curt_odom_.pose.pose.orientation).z;
    double delta_angle = curt_real_angle - prev_real_angle;
    if (delta_angle > M_PI) {delta_angle -= 2.0 * M_PI;}
    if (delta_angle < -M_PI) {delta_angle += 2.0 * M_PI;}
    moved_angle += std::abs(delta_angle);
    prev_real_angle = curt_real_angle;

    auto feedback = std::make_shared<MoveWheelRotate::Feedback>();
    feedback->current_yaw = moved_angle * (goal->target_yaw / std::abs(goal->target_yaw));
    feedback->move_time.sec = (now() - start_time).seconds();
    feedback->move_time.nanosec = (now() - start_time).nanoseconds() % static_cast<int64_t>(1e9);
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  pub_cmd_vel_->publish(zero_vel);
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (now() - start_time).nanoseconds() %
    static_cast<int64_t>(1e9);
  goal_handle->succeed(result);
}

void WheelActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  curt_odom_ = *msg;
}

} // namespace sobit_home

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::WheelActionServer)
