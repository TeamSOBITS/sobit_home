#include "sobit_home_library/sobit_home_wheel_action_server.hpp"

namespace sobit_home
{

  WheelActionServer::WheelActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("wheel_action_server", options)
  {
    // Configure the QoS profile
    rclcpp::QoS qos_profile(1); // depth = 1
    // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    // qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    // qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Declare parameters (PID values)
    this->declare_parameter("wheel_linear_kp", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("wheel_linear_ki", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("wheel_linear_kd", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("wheel_rotate_kp", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("wheel_rotate_ki", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("wheel_rotate_kd", rclcpp::PARAMETER_DOUBLE);

    // Retrieve parameters (PID values)
    wheel_linear_kp_ = this->get_parameter("wheel_linear_kp").as_double();
    wheel_linear_ki_ = this->get_parameter("wheel_linear_ki").as_double();
    wheel_linear_kd_ = this->get_parameter("wheel_linear_kd").as_double();

    wheel_rotate_kp_ = this->get_parameter("wheel_rotate_kp").as_double();
    wheel_rotate_ki_ = this->get_parameter("wheel_rotate_ki").as_double();
    wheel_rotate_kd_ = this->get_parameter("wheel_rotate_kd").as_double();

    // Log the parameters
    RCLCPP_INFO(this->get_logger(), "Wheel Linear PID parameters:");
    RCLCPP_INFO(this->get_logger(), "  Kp: %f", wheel_linear_kp_);
    RCLCPP_INFO(this->get_logger(), "  Ki: %f", wheel_linear_ki_);
    RCLCPP_INFO(this->get_logger(), "  Kd: %f", wheel_linear_kd_);

    RCLCPP_INFO(this->get_logger(), "Wheel Rotate PID parameters:");
    RCLCPP_INFO(this->get_logger(), "  Kp: %f", wheel_rotate_kp_);
    RCLCPP_INFO(this->get_logger(), "  Ki: %f", wheel_rotate_ki_);
    RCLCPP_INFO(this->get_logger(), "  Kd: %f", wheel_rotate_kd_);

    // Action Servers
    this->action_server_move_wheel_linear_ = rclcpp_action::create_server<MoveWheelLinear>(
        this,
        "move_wheel_linear",
        std::bind(&WheelActionServer::handle_move_wheel_linear_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&WheelActionServer::handle_move_wheel_linear_cancel, this, std::placeholders::_1),
        std::bind(&WheelActionServer::handle_move_wheel_linear_accepted, this, std::placeholders::_1));
    this->action_server_move_wheel_rotate_ = rclcpp_action::create_server<MoveWheelRotate>(
        this,
        "move_wheel_rotate",
        std::bind(&WheelActionServer::handle_move_wheel_rotate_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&WheelActionServer::handle_move_wheel_rotate_cancel, this, std::placeholders::_1),
        std::bind(&WheelActionServer::handle_move_wheel_rotate_accepted, this, std::placeholders::_1));

    // Publisher and Subscriber
    this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", qos_profile);
    this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos_profile, std::bind(&WheelActionServer::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "WheelActionServer has been initialized.");
  }
  WheelActionServer::~WheelActionServer()
  {
    this->action_server_move_wheel_linear_.reset();
    this->action_server_move_wheel_rotate_.reset();

    this->pub_cmd_vel_.reset();
    this->sub_odom_.reset();

    RCLCPP_INFO(this->get_logger(), "WheelActionServer has been terminated.");
  }

  rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_linear_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveWheelLinear::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_rotate_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveWheelRotate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_linear_cancel(
      const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_rotate_cancel(
      const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void WheelActionServer::handle_move_wheel_linear_accepted(
      const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)goal_handle;
    std::thread{std::bind(&WheelActionServer::exe_move_wheel_linear, this, std::placeholders::_1), goal_handle}.detach();
  }
  void WheelActionServer::handle_move_wheel_rotate_accepted(
      const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)goal_handle;
    std::thread{std::bind(&WheelActionServer::exe_move_wheel_rotate, this, std::placeholders::_1), goal_handle}.detach();
  }

  // TODO: goal time allowance is not considered
  void WheelActionServer::exe_move_wheel_linear(
      const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveWheelLinear::Result>();

    // Check if the odometry is updated
    // while (this->curt_odom_.header.stamp == this->init_odom_.header.stamp) {
    //   RCLCPP_INFO(this->get_logger(), "Waiting for the odometry to be updated");
    //   rclcpp::spin_some(this->get_node_base_interface());
    // }

    // Check if the target point is valid (only x is considered)
    if (goal->target_point.z != 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid target point: (%f, %f, %f)",
                   goal->target_point.x, goal->target_point.y, goal->target_point.z);
      result->success = false;
      result->message = "[FAIL] Invalid target point";
      goal_handle->abort(result);
      return;
    }

    // Initialize values
    geometry_msgs::msg::Twist init_vel, out_vel;
    double goal_dist = std::sqrt(std::pow(goal->target_point.x, 2) + std::pow(goal->target_point.y, 2));
    double curt_dist = 0.0;

    // PID state
    double prev_error = goal_dist;
    double error_integral = 0.0;

    // Set current time
    auto start_time = this->now();
    auto prev_time = start_time;
    this->init_odom_ = this->curt_odom_;
    rclcpp::Rate loop_rate(10);

    while (curt_dist < goal_dist)
    {
      // Check if the goal has been canceled
      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
        this->pub_cmd_vel_->publish(init_vel);

        result->success = false;
        result->message = "[FAIL] Goal has been canceled";
        result->total_elapsed_time.sec = (this->now() - start_time).seconds();
        result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

        goal_handle->canceled(result);
        return;
      }

      // Get the current time and compute dt
      auto curt_time = this->now();
      double dt = (curt_time - prev_time).nanoseconds() / 1e9;
      if (dt <= 0.0) dt = 0.1;  // guard against zero on first tick
      prev_time = curt_time;

      double vel_linear = 0.0;

      // Standard discrete PID
      double error = goal_dist - curt_dist;
      error_integral += error * dt;
      double error_derivative = (error - prev_error) / dt;
      prev_error = error;

      vel_linear = wheel_linear_kp_ * error
                 + wheel_linear_ki_ * error_integral
                 + wheel_linear_kd_ * error_derivative;

      // Calculate the output velocity
      out_vel.linear.x = vel_linear * std::cos(std::atan2(goal->target_point.y, goal->target_point.x));
      out_vel.linear.y = vel_linear * std::sin(std::atan2(goal->target_point.y, goal->target_point.x));

      // Publish the velocity
      this->pub_cmd_vel_->publish(out_vel);

      // Update the previous error
      curt_dist = std::sqrt(
          std::pow(this->curt_odom_.pose.pose.position.x - this->init_odom_.pose.pose.position.x, 2) +
          std::pow(this->curt_odom_.pose.pose.position.y - this->init_odom_.pose.pose.position.y, 2));

      // Publish feedback
      auto feedback = std::make_shared<MoveWheelLinear::Feedback>();
      feedback->current_point.x = curt_dist * std::cos(std::atan2(goal->target_point.y, goal->target_point.x));
      feedback->current_point.y = curt_dist * std::sin(std::atan2(goal->target_point.y, goal->target_point.x));
      feedback->move_time.sec = (this->now() - start_time).seconds();
      feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->publish_feedback(feedback);

      //
      loop_rate.sleep();
    }

    // Stop the robot
    this->pub_cmd_vel_->publish(init_vel);

    // Publish the result
    result->success = true;
    result->message = "Goal has been succeeded";
    result->total_elapsed_time.sec = (this->now() - start_time).seconds();
    result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->succeed(result);
  }

  // TODO: goal time allowance is not considered
  void WheelActionServer::exe_move_wheel_rotate(
      const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveWheelRotate::Result>();

    // Initialize values
    this->init_odom_ = this->curt_odom_;
    double init_real_angle = this->get_euler_from_quat(this->init_odom_.pose.pose.orientation).z;
    double curt_real_angle = this->get_euler_from_quat(this->curt_odom_.pose.pose.orientation).z;
    double prev_real_angle = init_real_angle;

    geometry_msgs::msg::Twist out_vel;
    geometry_msgs::msg::Twist zero_vel;
    double moved_angle = 0.0;
    double goal_angle = std::abs(goal->target_yaw);

    // PID state
    double prev_error_rot = goal_angle;
    double error_integral_rot = 0.0;
    double max_angular_speed = 0.7;

    // Set current time
    auto start_time = this->now();
    auto prev_time_rot = start_time;
    rclcpp::Rate loop_rate(10);

    while (moved_angle < goal_angle)
    {
      // Check if the goal has been canceled
      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
        this->pub_cmd_vel_->publish(zero_vel);

        result->success = false;
        result->message = "[FAIL] Goal has been canceled";
        result->total_elapsed_time.sec = (this->now() - start_time).seconds();
        result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
        goal_handle->canceled(result);

        return;
      }

      // Get the current time and compute dt
      auto curt_time = this->now();
      double dt_rot = (curt_time - prev_time_rot).nanoseconds() / 1e9;
      if (dt_rot <= 0.0) dt_rot = 0.1;
      prev_time_rot = curt_time;

      // Standard discrete PID
      double error_rot = goal_angle - moved_angle;
      error_integral_rot += error_rot * dt_rot;
      double error_derivative_rot = (error_rot - prev_error_rot) / dt_rot;
      prev_error_rot = error_rot;

      double vel_angular = wheel_rotate_kp_ * error_rot
                         + wheel_rotate_ki_ * error_integral_rot
                         + wheel_rotate_kd_ * error_derivative_rot;

      // Apply the maximum speed limit
      vel_angular = (goal->target_yaw > 0) ? std::min(vel_angular, max_angular_speed) : -std::min(std::abs(vel_angular), max_angular_speed);
      out_vel.angular.z = vel_angular;

      // Publish the velocity
      this->pub_cmd_vel_->publish(out_vel);

      // Calculate the moved distance
      curt_real_angle = this->get_euler_from_quat(this->curt_odom_.pose.pose.orientation).z;

      double delta_angle = curt_real_angle - prev_real_angle;

      if (delta_angle > M_PI)
        delta_angle -= 2 * M_PI;
      else if (delta_angle < -M_PI)
        delta_angle += 2 * M_PI;

      moved_angle += std::abs(delta_angle);
      prev_real_angle = curt_real_angle;

      // Publish feedback
      auto feedback = std::make_shared<MoveWheelRotate::Feedback>();
      feedback->current_yaw = moved_angle * (goal->target_yaw / std::abs(goal->target_yaw));
      feedback->move_time.sec = (this->now() - start_time).seconds();
      feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->publish_feedback(feedback);

      // Spin the node
      // rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
    }

    this->pub_cmd_vel_->publish(zero_vel);

    // Publish the result
    result->success = true;
    result->message = "Goal has been succeeded";
    result->total_elapsed_time.sec = (this->now() - start_time).seconds();
    result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->succeed(result);
  }

  void WheelActionServer::odom_callback(
      const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received odometry");

    this->curt_odom_ = *msg;

    RCLCPP_DEBUG(this->get_logger(), "Current odometry:");
    RCLCPP_DEBUG(this->get_logger(), "  Position: (%f, %f, %f)",
                 this->curt_odom_.pose.pose.position.x,
                 this->curt_odom_.pose.pose.position.y,
                 this->curt_odom_.pose.pose.position.z);
    RCLCPP_DEBUG(this->get_logger(), "  Orientation: (%f, %f, %f, %f)",
                 this->curt_odom_.pose.pose.orientation.x,
                 this->curt_odom_.pose.pose.orientation.y,
                 this->curt_odom_.pose.pose.orientation.z,
                 this->curt_odom_.pose.pose.orientation.w);
  }

} // namespace sobit_home

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::WheelActionServer)