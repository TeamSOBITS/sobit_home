#include "sobit_home_library/sobit_home_arm_teleop.hpp"
#include <rclcpp/parameter_client.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <Eigen/Geometry>
#include <thread>
#include <chrono>

namespace sobit_home
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MoveitArmTeleop::MoveitArmTeleop(const rclcpp::NodeOptions & options)
: Node(
    "moveit_arm_teleop",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---------- Read global tuning parameters ----------
  if (!this->has_parameter("arm_teleop.update_rate_hz"))
    this->declare_parameter("arm_teleop.update_rate_hz", 15.0);
  if (!this->has_parameter("arm_teleop.max_cartesian_step_m"))
    this->declare_parameter("arm_teleop.max_cartesian_step_m", 0.03);
  if (!this->has_parameter("arm_teleop.min_cartesian_fraction"))
    this->declare_parameter("arm_teleop.min_cartesian_fraction", 0.5);
  if (!this->has_parameter("arm_teleop.arrival_threshold_m"))
    this->declare_parameter("arm_teleop.arrival_threshold_m", 0.01);
  if (!this->has_parameter("arm_teleop.velocity_scaling"))
    this->declare_parameter("arm_teleop.velocity_scaling", 0.3);
  if (!this->has_parameter("arm_teleop.acceleration_scaling"))
    this->declare_parameter("arm_teleop.acceleration_scaling", 0.3);
  if (!this->has_parameter("arm_teleop.eef_step_m"))
    this->declare_parameter("arm_teleop.eef_step_m", 0.03);
  if (!this->has_parameter("arm_teleop.replan_threshold_m"))
    this->declare_parameter("arm_teleop.replan_threshold_m", 0.02);
  if (!this->has_parameter("arm_teleop.traj_lookahead_ms"))
    this->declare_parameter("arm_teleop.traj_lookahead_ms", 50);
  if (!this->has_parameter("arm_teleop.ompl_planning_timeout_s"))
    this->declare_parameter("arm_teleop.ompl_planning_timeout_s", 0.5);

  update_rate_hz_           = this->get_parameter("arm_teleop.update_rate_hz").as_double();
  max_cartesian_step_m_     = this->get_parameter("arm_teleop.max_cartesian_step_m").as_double();
  min_cartesian_fraction_   = this->get_parameter("arm_teleop.min_cartesian_fraction").as_double();
  arrival_threshold_m_      = this->get_parameter("arm_teleop.arrival_threshold_m").as_double();
  velocity_scaling_         = this->get_parameter("arm_teleop.velocity_scaling").as_double();
  acceleration_scaling_     = this->get_parameter("arm_teleop.acceleration_scaling").as_double();
  eef_step_m_               = this->get_parameter("arm_teleop.eef_step_m").as_double();
  replan_threshold_m_       = this->get_parameter("arm_teleop.replan_threshold_m").as_double();
  traj_lookahead_ms_        = this->get_parameter("arm_teleop.traj_lookahead_ms").as_int();
  ompl_planning_timeout_s_  = this->get_parameter("arm_teleop.ompl_planning_timeout_s").as_double();

  // ---------- Read per-arm configs ----------
  if (!this->has_parameter("arm_teleop.arms"))
    this->declare_parameter("arm_teleop.arms",
      std::vector<std::string>{"arm_left", "arm_right"});

  auto arm_names = this->get_parameter("arm_teleop.arms").as_string_array();

  for (const auto & arm_name : arm_names) {
    auto pg_key   = "arm_teleop." + arm_name + ".planning_group";
    auto tf_key   = "arm_teleop." + arm_name + ".target_frame";
    auto bf_key   = "arm_teleop." + arm_name + ".base_frame";
    auto traj_key = "arm_teleop." + arm_name + ".trajectory_topic";

    if (!this->has_parameter(pg_key))
      this->declare_parameter(pg_key, arm_name);
    if (!this->has_parameter(tf_key))
      this->declare_parameter(tf_key, arm_name == "arm_left" ? "left_target_link" : "right_target_link");
    if (!this->has_parameter(bf_key))
      this->declare_parameter(bf_key, "base_footprint");
    if (!this->has_parameter(traj_key))
      this->declare_parameter(traj_key,
        arm_name + "_position_controller/joint_trajectory");

    ArmTeleopConfig cfg;
    cfg.planning_group    = this->get_parameter(pg_key).as_string();
    cfg.target_frame      = this->get_parameter(tf_key).as_string();
    cfg.base_frame        = this->get_parameter(bf_key).as_string();
    cfg.trajectory_topic  = this->get_parameter(traj_key).as_string();

    auto arm_data = std::make_unique<ArmData>();
    arm_data->config = cfg;

    arm_data->traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      cfg.trajectory_topic, rclcpp::QoS(1));

    arms_[arm_name] = std::move(arm_data);

    auto sub = this->create_subscription<std_msgs::msg::Bool>(
      arm_name + "/moveit_track_enabled",
      rclcpp::QoS(1),
      [this, arm_name](const std_msgs::msg::Bool::SharedPtr msg) {
        enable_callback(arm_name, msg);
      });
    enable_subs_.push_back(sub);

    RCLCPP_INFO(get_logger(),
      "Arm '%s': target_frame='%s', base_frame='%s', traj_topic='%s'",
      arm_name.c_str(), cfg.target_frame.c_str(),
      cfg.base_frame.c_str(), cfg.trajectory_topic.c_str());
  }

  RCLCPP_INFO(get_logger(),
    "MoveitArmTeleop: rate=%.1f Hz, max_step=%.3f m, eef_step=%.3f m, "
    "replan_thresh=%.3f m, vel_scale=%.2f, lookahead=%d ms, ompl_timeout=%.2f s",
    update_rate_hz_, max_cartesian_step_m_, eef_step_m_,
    replan_threshold_m_, velocity_scaling_, traj_lookahead_ms_,
    ompl_planning_timeout_s_);

  init_thread_ = std::thread([this]() { init_move_groups(); });
}

MoveitArmTeleop::~MoveitArmTeleop()
{
  for (auto & [name, arm] : arms_) {
    arm->enabled = false;
  }
  for (auto & [name, arm] : arms_) {
    if (arm->thread.joinable()) {
      arm->thread.join();
    }
  }
  if (init_thread_.joinable()) {
    init_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// MoveGroupInterface initialisation (background thread)
// ---------------------------------------------------------------------------

void MoveitArmTeleop::init_move_groups()
{
  std::string ns = this->get_namespace();
  if (!ns.empty() && ns.front() == '/') {
    ns = ns.substr(1);
  }

  if (!this->has_parameter("robot_description")) {
    RCLCPP_INFO(get_logger(),
      "Fetching robot_description from /%s/move_group ...", ns.c_str());

    auto tmp_node = std::make_shared<rclcpp::Node>(
      "moveit_arm_teleop_param_fetch", this->get_namespace());
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
      tmp_node, "/" + ns + "/move_group");

    if (!param_client->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(),
        "move_group parameter service not available — arm teleop will be inactive");
      return;
    }

    auto base_params = param_client->get_parameters(
      {"robot_description", "robot_description_semantic"});

    if (base_params.empty() ||
        base_params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        base_params[0].as_string().empty()) {
      RCLCPP_ERROR(get_logger(),
        "robot_description is empty — arm teleop will be inactive");
      return;
    }

    this->declare_parameter("robot_description", base_params[0].as_string());
    if (base_params.size() >= 2 &&
        base_params[1].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      this->declare_parameter("robot_description_semantic", base_params[1].as_string());
    }
    RCLCPP_INFO(get_logger(),
      "robot_description declared (%zu chars)", base_params[0].as_string().size());

    rcl_interfaces::msg::ListParametersResult all_param_names;
    constexpr uint64_t DEPTH_RECURSIVE = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
      all_param_names = param_client->list_parameters({"robot_description_planning"}, DEPTH_RECURSIVE);
      if (!all_param_names.names.empty()) break;
      RCLCPP_INFO(get_logger(),
        "Waiting for robot_description_planning params on move_group (attempt %d/20)...",
        attempt + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(get_logger(),
      "robot_description_planning: found %zu param names via list_parameters",
      all_param_names.names.size());

    if (!all_param_names.names.empty()) {
      const auto & names = all_param_names.names;
      size_t copied = 0;
      for (const auto & pname : names) {
        auto result = param_client->get_parameters({pname});
        if (result.empty()) continue;
        const auto & p = result[0];
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) continue;
        try {
          if (!this->has_parameter(p.get_name())) {
            this->declare_parameter(p.get_name(), p.get_parameter_value());
            ++copied;
          }
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        } catch (const std::exception & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "declare_parameter failed for '%s': %s", p.get_name().c_str(), e.what());
        }
      }
      RCLCPP_INFO(get_logger(),
        "robot_description_planning: copied %zu / %zu sub-parameters from move_group",
        copied, names.size());
    } else {
      RCLCPP_WARN(get_logger(),
        "robot_description_planning namespace empty on move_group — TOTG may fail");
    }
  }

  for (auto & [arm_name, arm_data] : arms_) {
    if (arm_data->mgi) continue;
    try {
      moveit::planning_interface::MoveGroupInterface::Options opts(
        arm_data->config.planning_group,
        "robot_description",
        "/" + ns);

      auto mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        opts,
        tf_buffer_,
        rclcpp::Duration::from_seconds(30.0));

      mgi->setMaxVelocityScalingFactor(velocity_scaling_);
      mgi->setMaxAccelerationScalingFactor(acceleration_scaling_);

      if (mgi->getJoints().empty()) {
        RCLCPP_ERROR(get_logger(),
          "Arm '%s': no joints found in planning group — check SRDF", arm_name.c_str());
        continue;
      }

      arm_data->mgi = mgi;
      RCLCPP_INFO(get_logger(),
        "Arm '%s' ready: planning_frame='%s', ee_link='%s'",
        arm_name.c_str(),
        mgi->getPlanningFrame().c_str(),
        mgi->getEndEffectorLink().c_str());

      // ── Pre-build TOTG joint limit maps (cached for lifetime of tracking) ──
      // This avoids rebuilding the map on every control cycle (was ~1ms/cycle).
      const moveit::core::JointModelGroup * jmg =
        mgi->getRobotModel()->getJointModelGroup(arm_data->config.planning_group);
      if (jmg) {
        std::unordered_map<std::string, double> vel_limits, accel_limits;
        for (const auto * jm : jmg->getActiveJointModels()) {
          const std::string & jname = jm->getName();
          const std::string prefix = "robot_description_planning.joint_limits." + jname + ".";
          if (this->has_parameter(prefix + "max_velocity")) {
            vel_limits[jname] =
              this->get_parameter(prefix + "max_velocity").as_double() * velocity_scaling_;
          }
          if (this->has_parameter(prefix + "has_acceleration_limits") &&
              this->get_parameter(prefix + "has_acceleration_limits").as_bool() &&
              this->has_parameter(prefix + "max_acceleration")) {
            accel_limits[jname] =
              this->get_parameter(prefix + "max_acceleration").as_double() * acceleration_scaling_;
          }
        }
        arm_data->vel_limits   = std::move(vel_limits);
        arm_data->accel_limits = std::move(accel_limits);
        RCLCPP_INFO(get_logger(),
          "Arm '%s': cached TOTG limits for %zu joints",
          arm_name.c_str(), arm_data->vel_limits.size());
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "Failed to init MoveGroupInterface for '%s': %s", arm_name.c_str(), e.what());
    }
  }
}

// ---------------------------------------------------------------------------
// Enable / disable callback
// ---------------------------------------------------------------------------

void MoveitArmTeleop::enable_callback(
  const std::string & arm_name,
  const std_msgs::msg::Bool::SharedPtr msg)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) return;
  auto & arm = *it->second;

  if (!arm.mgi) {
    RCLCPP_WARN(get_logger(),
      "Arm '%s' MoveGroupInterface not ready yet — ignoring enable=true. "
      "Check that move_group is running and robot_description was fetched.",
      arm_name.c_str());
    return;
  }

  if (msg->data) {
    if (arm.enabled.load()) return;
    arm.enabled = true;

    if (arm.thread.joinable() && !arm.thread_active.load()) {
      arm.thread.join();
    }
    if (!arm.thread_active.load()) {
      arm.thread_active = true;
      arm.thread = std::thread([this, arm_name]() { tracking_loop(arm_name); });
      RCLCPP_INFO(get_logger(), "Arm tracking ENABLED for '%s'", arm_name.c_str());
    }
  } else {
    if (!arm.enabled.load()) return;
    arm.enabled = false;
    RCLCPP_INFO(get_logger(), "Arm tracking DISABLED for '%s'", arm_name.c_str());
  }
}

// ---------------------------------------------------------------------------
// Tracking loop (per-arm thread)
// ---------------------------------------------------------------------------

void MoveitArmTeleop::tracking_loop(const std::string & arm_name)
{
  auto & arm = *arms_[arm_name];
  auto & mgi = arm.mgi;
  const auto & cfg = arm.config;

  RCLCPP_INFO(get_logger(), "Tracking loop started for arm '%s'", arm_name.c_str());

  rclcpp::Rate rate(update_rate_hz_);

  geometry_msgs::msg::Pose last_submitted_target;
  bool first_iter = true;
  last_heartbeat_sec_ = this->now().seconds();

  // Lookahead duration added to trajectory stamp so the controller can
  // start interpolating immediately rather than waiting for the next cycle.
  const rclcpp::Duration lookahead =
    rclcpp::Duration::from_nanoseconds(traj_lookahead_ms_ * 1'000'000LL);

  while (rclcpp::ok() && arm.enabled.load()) {
    // ── 1. Look up target TF (non-blocking: timeout=0) ────────────────────
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        cfg.base_frame,
        cfg.target_frame,
        tf2::TimePointZero,
        tf2::Duration(0));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed for arm '%s': %s", arm_name.c_str(), e.what());
      rate.sleep();
      continue;
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x  = tf_stamped.transform.translation.x;
    target_pose.position.y  = tf_stamped.transform.translation.y;
    target_pose.position.z  = tf_stamped.transform.translation.z;
    target_pose.orientation = tf_stamped.transform.rotation;

    // ── 2. Get current EE pose via robot state monitor (no ROS round-trip) ─
    geometry_msgs::msg::Pose current_pose;
    try {
      current_pose = mgi->getCurrentPose().pose;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "getCurrentPose failed for '%s': %s", arm_name.c_str(), e.what());
      rate.sleep();
      continue;
    }

    // ── 3. Distance check ─────────────────────────────────────────────────
    double dist = pose_distance(current_pose, target_pose);
    if (dist < arrival_threshold_m_) {
      rate.sleep();
      continue;
    }

    // ── 4. Replan decision ────────────────────────────────────────────────
    // Replan whenever target moved > replan_threshold_m_ (tight tracking),
    // OR on heartbeat so the first publish is never silently dropped.
    auto now_sec = this->now().seconds();
    bool heartbeat = (now_sec - last_heartbeat_sec_ >= heartbeat_period_sec_);
    if (!first_iter && !heartbeat) {
      double target_moved = pose_distance(target_pose, last_submitted_target);
      if (target_moved < replan_threshold_m_) {
        rate.sleep();
        continue;
      }
    }
    if (heartbeat) last_heartbeat_sec_ = now_sec;

    // ── 5. Clamp step to max_cartesian_step_m_ ────────────────────────────
    geometry_msgs::msg::Pose step_target = target_pose;
    if (dist > max_cartesian_step_m_) {
      double scale = max_cartesian_step_m_ / dist;
      step_target.position.x = current_pose.position.x +
        (target_pose.position.x - current_pose.position.x) * scale;
      step_target.position.y = current_pose.position.y +
        (target_pose.position.y - current_pose.position.y) * scale;
      step_target.position.z = current_pose.position.z +
        (target_pose.position.z - current_pose.position.z) * scale;
      tf2::Quaternion q_c, q_t;
      tf2::fromMsg(current_pose.orientation, q_c);
      tf2::fromMsg(target_pose.orientation, q_t);
      step_target.orientation = tf2::toMsg(q_c.slerp(q_t, scale));
    }

    // ── 6. Compute Cartesian path ─────────────────────────────────────────
    // setStartStateToCurrentState() pulls joint state from the robot state
    // monitor — it is a local read, not a service call.
    mgi->setStartStateToCurrentState();
    std::vector<geometry_msgs::msg::Pose> waypoints = {step_target};
    moveit_msgs::msg::RobotTrajectory traj_msg;

    auto t0 = std::chrono::steady_clock::now();
    double fraction = mgi->computeCartesianPath(
      waypoints,
      eef_step_m_,   // interpolation resolution (tunable, default 3 cm)
      traj_msg,
      true           // avoid_collisions
    );
    auto plan_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();

    if (first_iter) {
      RCLCPP_INFO(get_logger(),
        "Arm '%s': first plan — dist=%.3f m, step=%.3f m, fraction=%.2f, plan=%ldms",
        arm_name.c_str(), dist,
        std::min(dist, max_cartesian_step_m_),
        fraction, plan_ms);
    } else {
      RCLCPP_DEBUG(get_logger(),
        "Arm '%s': dist=%.3f m, step=%.3f m, fraction=%.2f, plan=%ldms",
        arm_name.c_str(), dist,
        std::min(dist, max_cartesian_step_m_),
        fraction, plan_ms);
    }

    // Log a throttled INFO only when planning is slow (>control period)
    const double period_ms = 1000.0 / update_rate_hz_;
    if (plan_ms > static_cast<long>(period_ms)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Arm '%s': slow plan %ldms (target %.1fms), dist=%.3f m, fraction=%.2f",
        arm_name.c_str(), plan_ms, period_ms, dist, fraction);
    }

    if (fraction < min_cartesian_fraction_) {
      // ── 6b. Cartesian failed — fall back to OMPL joint-space plan ─────────
      // This handles transitions through singularities and out-of-workspace
      // regions that cannot be traversed in a straight Cartesian line.
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Arm '%s': Cartesian fraction %.2f < %.2f — falling back to OMPL",
        arm_name.c_str(), fraction, min_cartesian_fraction_);

      mgi->setPlanningTime(ompl_planning_timeout_s_);
      mgi->setPoseTarget(step_target);
      moveit::planning_interface::MoveGroupInterface::Plan ompl_plan;
      auto ompl_result = mgi->plan(ompl_plan);

      if (ompl_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "OMPL plan failed for arm '%s' (code %d)", arm_name.c_str(),
          ompl_result.val);
        rate.sleep();
        continue;
      }

      // Publish OMPL trajectory directly (already time-parameterized by planner)
      trajectory_msgs::msg::JointTrajectory jtraj_ompl =
        ompl_plan.trajectory.joint_trajectory;
      jtraj_ompl.header.stamp = this->now() + lookahead;
      arm.traj_pub->publish(jtraj_ompl);

      last_submitted_target = step_target;
      first_iter = false;

      rate.sleep();
      continue;
    }

    // ── 7. Time-parameterise with cached TOTG limits ───────────────────────
    auto robot_traj = std::make_shared<robot_trajectory::RobotTrajectory>(
      mgi->getRobotModel(), cfg.planning_group);
    robot_traj->setRobotTrajectoryMsg(*mgi->getCurrentState(), traj_msg);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    bool totg_ok = false;
    if (!arm.vel_limits.empty() && !arm.accel_limits.empty()) {
      // Use pre-built cached limit maps — no parameter lookup per cycle
      totg_ok = totg.computeTimeStamps(*robot_traj, arm.vel_limits, arm.accel_limits);
    } else {
      totg_ok = totg.computeTimeStamps(*robot_traj, velocity_scaling_, acceleration_scaling_);
    }

    if (!totg_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "TOTG failed for arm '%s'", arm_name.c_str());
      rate.sleep();
      continue;
    }
    robot_traj->getRobotTrajectoryMsg(traj_msg);

    // ── 8. Publish directly to joint trajectory controller ────────────────
    // Stamp with now() + lookahead so the controller can pre-buffer the
    // trajectory and start executing without waiting for the next cycle.
    trajectory_msgs::msg::JointTrajectory jtraj = traj_msg.joint_trajectory;
    jtraj.header.stamp = this->now() + lookahead;
    arm.traj_pub->publish(jtraj);

    last_submitted_target = step_target;
    first_iter = false;

    rate.sleep();
  }

  mgi->stop();
  arm.thread_active = false;
  RCLCPP_INFO(get_logger(), "Tracking loop ended for arm '%s'", arm_name.c_str());
}

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

double MoveitArmTeleop::pose_distance(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace sobit_home

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::MoveitArmTeleop)
