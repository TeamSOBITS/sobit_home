#include "sobit_home_library/sobit_home_moveit_server.hpp"
#include <atomic>
#include <rclcpp/parameter_client.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

namespace sobit_home
{

MoveitServer::MoveitServer(const rclcpp::NodeOptions & options)
: Node("moveit_server",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  plan_service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Service
  plan_service_ = create_service<PlanToPose>(
    "plan_to_pose",
    [this](
      const std::shared_ptr<PlanToPose::Request> req,
      std::shared_ptr<PlanToPose::Response> res) {
      handle_plan_request(req, res);
    },
    rclcpp::ServicesQoS(),
    plan_service_cb_group_);

  plan_to_named_pose_service_ = create_service<PlanToNamedPose>(
    "plan_to_named_pose",
    [this](
      const std::shared_ptr<PlanToNamedPose::Request> req,
      std::shared_ptr<PlanToNamedPose::Response> res) {
      handle_plan_to_named_pose_request(req, res);
    },
    rclcpp::ServicesQoS(),
    plan_service_cb_group_);

  execute_action_server_ = rclcpp_action::create_server<ExecutePlan>(
    this, "execute_plan",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecutePlan::Goal> goal) {
      return handle_exec_goal({}, goal);
    },
    [this](const std::shared_ptr<GoalHandleExecutePlan> goal_handle) {
      return handle_exec_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleExecutePlan> goal_handle) {
      handle_exec_accepted(goal_handle);
    },
    rcl_action_server_get_default_options(),
    action_cb_group_);

  cleanup_timer_ = create_wall_timer(
    std::chrono::seconds(10),
    [this]() {purge_stale_plans();});

  // Single-arm groups plus their whole-body counterparts, which move the base
  // through MoveitWholeBodyBridge. Override per launch to plan for other groups.
  if (!has_parameter("active_planning_groups")) {
    declare_parameter(
      "active_planning_groups",
      std::vector<std::string>{"arm_left", "arm_right", "arm_left_body", "arm_right_body"});
  }

  // Planning budget + workspace bounds
  init_tunable_params();

  RCLCPP_INFO(get_logger(), "MoveitServer starting (clock_type=%d)",
    static_cast<int>(get_clock()->get_clock_type()));

  init_timer_ = create_wall_timer(
    std::chrono::milliseconds(0),
    [this]() {
      init_timer_->cancel();
      if (!init_move_groups()) {
        schedule_init_retry();
      }
    });
}

MoveitServer::~MoveitServer() {}

void MoveitServer::schedule_init_retry()
{
  if (++init_attempts_ >= kMaxInitAttempts) {
    RCLCPP_ERROR(get_logger(),
      "Giving up on MoveGroupInterface init after %d attempts — "
      "planning services will not work", init_attempts_);
    return;
  }
  RCLCPP_WARN(get_logger(),
    "MoveGroupInterface init incomplete — retrying in 5s (attempt %d/%d)",
    init_attempts_, kMaxInitAttempts);
  init_timer_ = create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      init_timer_->cancel();
      if (!init_move_groups()) {
        schedule_init_retry();
      }
    });
}

bool MoveitServer::init_move_groups()
{
  const auto groups = get_parameter("active_planning_groups").as_string_array();

  std::string ns = get_namespace();
  if (!ns.empty() && ns.front() == '/') {
    ns = ns.substr(1);
  }

  const std::string move_group_service =
    ns.empty() ? "/move_group" : "/" + ns + "/move_group";

  if (!has_parameter("robot_description")) {
    RCLCPP_INFO(get_logger(), "Fetching robot_description from %s ...", move_group_service.c_str());
    try {
      auto tmp_node = std::make_shared<rclcpp::Node>("moveit_server_param_fetch", get_namespace());
      auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
        tmp_node, move_group_service);
      if (!param_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(get_logger(), "move_group parameter service not available yet");
        return false;
      }
      const auto params = param_client->get_parameters({"robot_description",
            "robot_description_semantic"});
      if (params.size() >= 2 && !params[0].as_string().empty()) {
        declare_parameter("robot_description", params[0].as_string());
        declare_parameter("robot_description_semantic", params[1].as_string());
        RCLCPP_INFO(get_logger(),
            "robot_description declared on this node (%zu chars), SRDF (%zu chars)",
          params[0].as_string().size(), params[1].as_string().size());
      } else {
        RCLCPP_WARN(get_logger(), "robot_description not available on move_group yet");
        return false;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to fetch robot_description: %s", e.what());
      return false;
    }
  }

  // MoveGroupInterface's ctor kills the process on an unknown group — validate first.
  moveit::core::RobotModelConstPtr robot_model;
  try {
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    robot_model = loader.getModel();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load robot model: %s", e.what());
  }
  if (!robot_model) {
    return false;
  }

  bool all_done = true;
  for (const auto & group_name : groups) {
    {
      std::lock_guard<std::mutex> lock(active_groups_mutex_);
      if (active_groups_.count(group_name)) {
        continue; // already initialized on a previous retry
      }
    }

    if (!robot_model->hasJointModelGroup(group_name)) {
      RCLCPP_ERROR(get_logger(),
        "Planning group '%s' is not defined in the loaded SRDF — skipping. "
        "Check active_planning_groups against the SRDF selected at launch "
        "(teleop and non-teleop use different SRDFs).",
        group_name.c_str());
      continue;
    }

    try {
      RCLCPP_INFO(this->get_logger(),
                  "Initializing MoveGroupInterface for '%s' (ns: '%s')",
                  group_name.c_str(), ns.c_str());

      // Pass shared_from_this() so MoveGroupInterface uses this registered node.
      moveit::planning_interface::MoveGroupInterface::Options opts(
        group_name,
        "robot_description",
        ns.empty() ? "/" : "/" + ns);
      auto mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(),
          opts,
          tf_buffer_,
          rclcpp::Duration::from_seconds(30.0));

      mgi->setPlanningTime(10.0);
      mgi->setNumPlanningAttempts(10);
      mgi->setMaxVelocityScalingFactor(0.1);
      mgi->setMaxAccelerationScalingFactor(0.1);

      if (mgi->getJoints().empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Group '%s' has no joints — is the SRDF loaded and move_group running?",
                     group_name.c_str());
        all_done = false;
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Initialized '%s'  planning_frame='%s'  ee_link='%s'",
                    group_name.c_str(),
                    mgi->getPlanningFrame().c_str(),
                    mgi->getEndEffectorLink().c_str());
        std::lock_guard<std::mutex> lock(active_groups_mutex_);
        active_groups_[group_name] = mgi;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to init MoveGroupInterface for '%s': %s",
                   group_name.c_str(), e.what());
      all_done = false;
    }
  }
  return all_done;
}

void MoveitServer::init_tunable_params()
{
  auto declare_double = [this](const std::string & name, double default_val,
      std::atomic<double> & sink) {
    const double v = has_parameter(name)
      ? get_parameter(name).as_double()
      : declare_parameter(name, default_val);
    sink.store(v);
  };

  declare_double("plan_time_sec", plan_time_sec_.load(), plan_time_sec_);

  const int attempts = has_parameter("plan_attempts")
    ? static_cast<int>(get_parameter("plan_attempts").as_int())
    : static_cast<int>(declare_parameter("plan_attempts", plan_attempts_.load()));
  plan_attempts_.store(attempts);

  declare_double("workspace_min_x", workspace_min_x_.load(), workspace_min_x_);
  declare_double("workspace_min_y", workspace_min_y_.load(), workspace_min_y_);
  declare_double("workspace_min_z", workspace_min_z_.load(), workspace_min_z_);
  declare_double("workspace_max_x", workspace_max_x_.load(), workspace_max_x_);
  declare_double("workspace_max_y", workspace_max_y_.load(), workspace_max_y_);
  declare_double("workspace_max_z", workspace_max_z_.load(), workspace_max_z_);

  RCLCPP_INFO(get_logger(),
    "Planning params: time=%.2fs attempts=%d workspace=[%.2f %.2f %.2f]..[%.2f %.2f %.2f]",
    plan_time_sec_.load(), plan_attempts_.load(),
    workspace_min_x_.load(), workspace_min_y_.load(), workspace_min_z_.load(),
    workspace_max_x_.load(), workspace_max_y_.load(), workspace_max_z_.load());

  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return on_set_parameters(params);
    });
}

rcl_interfaces::msg::SetParametersResult MoveitServer::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    const auto & name = p.get_name();

    if (name == "plan_attempts") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "plan_attempts must be an integer";
        break;
      }
      const int v = static_cast<int>(p.as_int());
      if (v < 1) {
        result.successful = false;
        result.reason = "plan_attempts must be >= 1";
        break;
      }
      plan_attempts_.store(v);
      continue;
    }

    // All remaining tunables are doubles.
    const std::unordered_map<std::string, std::atomic<double> *> double_sinks = {
      {"plan_time_sec", &plan_time_sec_},
      {"workspace_min_x", &workspace_min_x_},
      {"workspace_min_y", &workspace_min_y_},
      {"workspace_min_z", &workspace_min_z_},
      {"workspace_max_x", &workspace_max_x_},
      {"workspace_max_y", &workspace_max_y_},
      {"workspace_max_z", &workspace_max_z_},
    };
    auto it = double_sinks.find(name);
    if (it == double_sinks.end()) {
      continue;  // not one of ours — leave it to other callbacks/default handling
    }
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = name + " must be a double";
      break;
    }
    // TODO(min-max): each workspace bound is validated independently here, so
    // setting them one at a time can transiently leave min > max (e.g. set
    // workspace_min_x above the current max_x before lowering max_x).
    // setWorkspace() tolerates an inverted AABB, so we defer cross-field
    // coherence validation for now. Revisit if a degenerate workspace ever
    // produces planning failures.
    const double v = p.as_double();
    if (name == "plan_time_sec" && v <= 0.0) {
      result.successful = false;
      result.reason = "plan_time_sec must be > 0";
      break;
    }
    it->second->store(v);
  }

  return result;
}

void MoveitServer::handle_plan_request(
  const std::shared_ptr<PlanToPose::Request> request,
  std::shared_ptr<PlanToPose::Response> response)
{
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  {
    std::lock_guard<std::mutex> lock(active_groups_mutex_);
    if (active_groups_.empty()) {
      response->success = false;
      response->message = "MoveGroups are still initializing.";
      return;
    }
    auto it = active_groups_.find(request->planning_group);
    if (it == active_groups_.end()) {
      response->success = false;
      response->message = "Planning group not found: " + request->planning_group;
      return;
    }
    move_group = it->second;
  }

  RCLCPP_INFO(get_logger(), "Received planning request for group: %s",
      request->planning_group.c_str());

  try {
    RCLCPP_INFO(get_logger(), "Planning Frame: %s, Pose Frame: %s",
      move_group->getPlanningFrame().c_str(), request->target.header.frame_id.c_str());
    const auto current_pose = move_group->getCurrentPose();
    RCLCPP_INFO(get_logger(), "Current Pose: x:%.2f, y:%.2f, z:%.2f",
      current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Could not retrieve current pose: %s", e.what());
  }

  move_group->setPlanningTime(plan_time_sec_.load());
  move_group->setNumPlanningAttempts(plan_attempts_.load());
  move_group->setWorkspace(
    workspace_min_x_.load(), workspace_min_y_.load(), workspace_min_z_.load(),
    workspace_max_x_.load(), workspace_max_y_.load(), workspace_max_z_.load());
  {
    moveit::core::RobotStatePtr start_state = move_group->getCurrentState(2.0);
    if (start_state) {
      start_state->enforceBounds();
      move_group->setStartState(*start_state);
    } else {
      RCLCPP_WARN(get_logger(), "getCurrentState() returned null; falling back to current state.");
      move_group->setStartStateToCurrentState();
    }
  }

  if (request->target.header.frame_id.empty()) {
    request->target.header.frame_id = move_group->getPlanningFrame();
    RCLCPP_WARN(get_logger(), "Target frame_id was empty, using planning frame: %s",
      request->target.header.frame_id.c_str());
  }

  RCLCPP_INFO(get_logger(), "Setting Pose Target: x:%.2f, y:%.2f, z:%.2f",
    request->target.pose.position.x, request->target.pose.position.y,
      request->target.pose.position.z);

  if (!move_group->setPoseTarget(request->target)) {
    RCLCPP_ERROR(get_logger(), "Failed to set pose target (invalid pose?)");
    response->success = false;
    response->message = "Failed to set pose target.";
    return;
  }

  RCLCPP_INFO(get_logger(), "Starting OMPL planning...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::MoveItErrorCode plan_result = move_group->plan(my_plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "Initial Pose planning failed (%i). Relaxing tolerance...",
      static_cast<int>(plan_result.val));
    move_group->setGoalOrientationTolerance(0.1);
    plan_result = move_group->plan(my_plan);
  }

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(),
        "Relaxed Pose planning failed (%i). Attempting Position-Only fallback...",
      static_cast<int>(plan_result.val));
    move_group->clearPoseTargets();
    if (move_group->setPositionTarget(
        request->target.pose.position.x,
        request->target.pose.position.y,
        request->target.pose.position.z))
    {
      plan_result = move_group->plan(my_plan);
      RCLCPP_INFO(get_logger(), "Position-Only planning finished with code: %i",
        static_cast<int>(plan_result.val));
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set Position target even in fallback.");
    }
  }

  RCLCPP_INFO(get_logger(), "Final planning result code: %i", static_cast<int>(plan_result.val));

  if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
    const std::string plan_id = "plan_" + std::to_string(now().nanoseconds());
    {
      std::lock_guard<std::mutex> lock(plan_cache_mutex_);
      CachedPlan cp;
      cp.plan = my_plan;
      cp.timestamp = now();
      cp.planning_group = request->planning_group;
      plan_cache_[plan_id] = cp;
    }

    double duration = 0.0;
    if (!my_plan.trajectory.joint_trajectory.points.empty()) {
      const auto & pts = my_plan.trajectory.joint_trajectory.points;
      duration = pts.back().time_from_start.sec + pts.back().time_from_start.nanosec * 1e-9;
    }
    response->success = true;
    response->plan_id = plan_id;
    response->trajectory = my_plan.trajectory;
    response->estimated_time = duration;
    response->message = "Plan generated with ID: " + plan_id;
  } else {
    response->success = false;
    response->message = "MoveIt planning failed for both Pose and Position-Only strategies.";
    RCLCPP_INFO(get_logger(),
      "Hint: If planning fails with code 99999, check for self-collisions in the URDF/SRDF.");
  }
}

void MoveitServer::handle_plan_to_named_pose_request(
  const std::shared_ptr<PlanToNamedPose::Request> request,
  std::shared_ptr<PlanToNamedPose::Response> response)
{
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  {
    std::lock_guard<std::mutex> lock(active_groups_mutex_);
    if (active_groups_.empty()) {
      response->success = false;
      response->message = "MoveGroups are still initializing.";
      return;
    }
    auto it = active_groups_.find(request->planning_group);
    if (it == active_groups_.end()) {
      response->success = false;
      response->message = "Planning group not found: " + request->planning_group;
      return;
    }
    move_group = it->second;
  }

  RCLCPP_INFO(get_logger(), "Received planning request for group: %s",
      request->planning_group.c_str());

  try {
    const std::vector<std::string> joint_names = move_group->getJointNames();
    const std::vector<double> current_joint_values = move_group->getCurrentJointValues();
    
    std::string joint_log = "Current Joint Values: ";
    for (size_t i = 0; i < joint_names.size() && i < current_joint_values.size(); ++i) {
      joint_log += joint_names[i] + ":" + std::to_string(current_joint_values[i]) + " ";
    }
    RCLCPP_INFO(get_logger(), "%s", joint_log.c_str());
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Could not retrieve current joint values: %s", e.what());
  }

  move_group->setPlanningTime(10.0);
  move_group->setNumPlanningAttempts(10);
  move_group->setWorkspace(-5.0, -5.0, 0.0, 5.0, 5.0, 5.0);
  move_group->setStartStateToCurrentState();

  if (request->pose_name.empty()) {
    RCLCPP_WARN(get_logger(), "Pose name is empty");
    response->success = false;
    response->message = "Pose name is empty.";
    return;
  }

  RCLCPP_INFO(get_logger(), "Setting Pose Name: %s", request->pose_name.c_str());

  if (!move_group->setNamedTarget(request->pose_name)) {
    RCLCPP_ERROR(get_logger(), "Failed to set pose name '%s'.", request->pose_name.c_str());
    response->success = false;
    response->message = "Failed to set pose name: " + request->pose_name;
    return;
  }

  RCLCPP_INFO(get_logger(), "Starting OMPL planning...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::MoveItErrorCode plan_result = move_group->plan(my_plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "Initial Pose planning failed (%i). Relaxing tolerance...",
      static_cast<int>(plan_result.val));
      response->success = false;
      response->message = "Initial Pose planning failed.";
  }

  RCLCPP_INFO(get_logger(), "Final planning result code: %i", static_cast<int>(plan_result.val));

  if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
    const std::string plan_id = "plan_" + std::to_string(now().nanoseconds());
    {
      std::lock_guard<std::mutex> lock(plan_cache_mutex_);
      CachedPlan cp;
      cp.plan = my_plan;
      cp.timestamp = now();
      cp.planning_group = request->planning_group;
      plan_cache_[plan_id] = cp;
    }

    double duration = 0.0;
    if (!my_plan.trajectory.joint_trajectory.points.empty()) {
      const auto & pts = my_plan.trajectory.joint_trajectory.points;
      duration = pts.back().time_from_start.sec + pts.back().time_from_start.nanosec * 1e-9;
    }
    response->success = true;
    response->plan_id = plan_id;
    response->trajectory = my_plan.trajectory;
    response->estimated_time = duration;
    response->message = "Plan generated with ID: " + plan_id;
  } else {
    response->success = false;
    response->message = "MoveIt planning failed for both Pose and Position-Only strategies.";
    RCLCPP_INFO(get_logger(),
      "Hint: If planning fails with code 99999, check for self-collisions in the URDF/SRDF.");
  }

  RCLCPP_INFO(get_logger(), "Final planning result code: %i", static_cast<int>(plan_result.val));
}

rclcpp_action::GoalResponse MoveitServer::handle_exec_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecutePlan::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "handle_exec_goal called for plan_id: %s", goal->plan_id.c_str());
  std::lock_guard<std::mutex> lock(plan_cache_mutex_);
  if (plan_cache_.find(goal->plan_id) == plan_cache_.end()) {
    RCLCPP_ERROR(get_logger(), "Plan ID not found or expired: %s", goal->plan_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Goal ACCEPTED for plan_id: %s", goal->plan_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveitServer::handle_exec_cancel(
  const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Received request to cancel execution");

  std::string group_to_stop;
  {
    std::lock_guard<std::mutex> lock(plan_cache_mutex_);
    const std::string plan_id = goal_handle->get_goal()->plan_id;
    auto it = active_executions_.find(plan_id);
    if (it != active_executions_.end()) {
      group_to_stop = it->second;
    }
  }

  if (!group_to_stop.empty()) {
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi;
    {
      std::lock_guard<std::mutex> lock(active_groups_mutex_);
      auto it = active_groups_.find(group_to_stop);
      if (it != active_groups_.end()) {
        mgi = it->second;
      }
    }
    if (mgi) {
      mgi->stop();
    }
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitServer::handle_exec_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  std::thread{[this, goal_handle]() {execute_plan_thread(goal_handle);}}.detach();
}

void MoveitServer::execute_plan_thread(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  auto result = std::make_shared<ExecutePlan::Result>();
  std::string plan_id = goal_handle->get_goal()->plan_id;

  CachedPlan cached_plan;

  // 1. Retrieve the plan from the cache
  {
    std::lock_guard<std::mutex> lock(plan_cache_mutex_);
    auto it = plan_cache_.find(plan_id);
    if (it == plan_cache_.end()) {
      result->success = false;
      result->message = "Plan ID not found during execute";
      goal_handle->abort(result);
      return;
    }
    cached_plan = it->second;
    plan_cache_.erase(it);
  }

  // 2. Find the MoveGroup
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  {
    std::lock_guard<std::mutex> lock(active_groups_mutex_);
    auto move_group_it = active_groups_.find(cached_plan.planning_group);
    if (move_group_it == active_groups_.end()) {
      result->success = false;
      result->message = "Planning group lost";
      goal_handle->abort(result);
      return;
    }
    move_group = move_group_it->second;
  }

  // 3. Register this execution so handle_exec_cancel knows which arm to stop
  {
    std::lock_guard<std::mutex> lock(plan_cache_mutex_);
    active_executions_[plan_id] = cached_plan.planning_group;
  }

  // 4. Setup the Feedback Thread
  std::atomic<bool> is_executing{true};

  std::thread feedback_thread([this, goal_handle, move_group, &is_executing]() {
      rclcpp::Rate loop_rate(10); // Publish feedback at 10 Hz

      while (is_executing && rclcpp::ok()) {
      // If the goal is canceled, break out early
        if (goal_handle->is_canceling()) {
          break;
        }

        auto feedback = std::make_shared<ExecutePlan::Feedback>();
        feedback->current_state = "EXECUTING";

      // Optional: Calculate distance to goal
      // try {
      //   auto current_pose = move_group->getCurrentPose().pose;
      //   // Do some math to calculate Euclidean distance to the target pose here
      //   // feedback->distance_to_goal = ...
      // } catch (...) {}

        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }
    });

  // 5. Execute the plan (This is a BLOCKING call)
  moveit::core::MoveItErrorCode exec_result;
  try {
    exec_result = move_group->execute(cached_plan.plan);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during execute: %s", e.what());
    exec_result = moveit::core::MoveItErrorCode::FAILURE;
  }

  // 6. Stop the feedback thread safely
  is_executing = false;
  if (feedback_thread.joinable()) {
    feedback_thread.join();
  }

  // 7. Cleanup: Remove from active executions
  {
    std::lock_guard<std::mutex> lock(plan_cache_mutex_);
    active_executions_.erase(plan_id);
  }

  // 8. Handle results
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Execution was canceled";
    goal_handle->canceled(result);
    return;
  }

  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = true;
    result->message = "Execution successful";
    goal_handle->succeed(result);
  } else {
    result->success = false;
    result->message = "Execution failed";
    goal_handle->abort(result);
  }
}

void MoveitServer::purge_stale_plans()
{
  std::lock_guard<std::mutex> lock(plan_cache_mutex_);
  const auto now = this->now();
  for (auto it = plan_cache_.begin(); it != plan_cache_.end(); ) {
    if ((now - it->second.timestamp).seconds() > 60.0) {
      RCLCPP_WARN(get_logger(), "Purging expired plan_id: %s", it->first.c_str());
      it = plan_cache_.erase(it);
    } else {
      ++it;
    }
  }
}


}  // namespace sobit_home


RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::MoveitServer)
