#ifndef SOBIT_HOME_LIBRARY__SOBIT_HOME_MOVEIT_SERVER_HPP_
#define SOBIT_HOME_LIBRARY__SOBIT_HOME_MOVEIT_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "sobits_interfaces/srv/plan_to_pose.hpp"
#include "sobits_interfaces/action/execute_plan.hpp"

namespace sobit_home
{

struct CachedPlan
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  rclcpp::Time timestamp;
  std::string planning_group;
};

class MoveitServer : public rclcpp::Node
{
public:
  using PlanToPose = sobits_interfaces::srv::PlanToPose;
  using ExecutePlan = sobits_interfaces::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;

  explicit MoveitServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MoveitServer();

private:
  void handle_plan_request(
    const std::shared_ptr<PlanToPose::Request> request,
    std::shared_ptr<PlanToPose::Response> response);

  rclcpp_action::GoalResponse handle_exec_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecutePlan::Goal> goal);

  rclcpp_action::CancelResponse handle_exec_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  void handle_exec_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  void execute_plan_thread(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  void purge_stale_plans();

  void init_move_groups();

  // Declares the tunable planning parameters and wires up the dynamic-update
  // callback so they can be changed at runtime with `ros2 param set`.
  void init_tunable_params();

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);


  // One-shot timer that fires init_move_groups() after the constructor returns,
  // guaranteeing shared_from_this() is valid when MoveGroupInterface is built.
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Core map mapping string -> MoveGroupInterface
  // Both active_groups_ and active_executions_ are guarded by their respective mutexes.
  std::unordered_map<std::string,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> active_groups_;
  mutable std::mutex active_groups_mutex_;

  std::unordered_map<std::string, std::string> active_executions_; // plan_id -> planning_group

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Callback groups — must be members to prevent destruction at end of constructor
  rclcpp::CallbackGroup::SharedPtr plan_service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;

  // Service & Action
  rclcpp::Service<PlanToPose>::SharedPtr plan_service_;
  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_action_server_;

  // Plan Execution Cache
  std::unordered_map<std::string, CachedPlan> plan_cache_;
  std::mutex plan_cache_mutex_;
  rclcpp::TimerBase::SharedPtr cleanup_timer_;

  // Planning budget
  std::atomic<double> plan_time_sec_{3.5};
  std::atomic<int> plan_attempts_{4};

  // Planning workspace AABB applied to every plan request (min/max in the
  // planning frame). Atomic for the same cross-thread reason as above.
  std::atomic<double> workspace_min_x_{-5.0};
  std::atomic<double> workspace_min_y_{-5.0};
  std::atomic<double> workspace_min_z_{0.0};
  std::atomic<double> workspace_max_x_{5.0};
  std::atomic<double> workspace_max_y_{5.0};
  std::atomic<double> workspace_max_z_{5.0};

  // Keeps the dynamic-parameter callback registration alive.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace sobit_home

#endif  // SOBIT_HOME_LIBRARY__SOBIT_HOME_MOVEIT_SERVER_HPP_
