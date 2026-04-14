#ifndef SOBIT_HOME_LIBRARY__SOBIT_HOME_MOVEIT_SERVER_HPP_
#define SOBIT_HOME_LIBRARY__SOBIT_HOME_MOVEIT_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <mutex>
#include <unordered_map>

#include "sobits_interfaces/srv/plan_to_pose.hpp"
#include "sobits_interfaces/action/execute_plan.hpp"

namespace sobit_home
{

struct CachedPlan {
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


  // Background thread for blocking MoveGroupInterface construction
  std::thread init_thread_;

  // Core map mapping string -> MoveGroupInterface
  // Both active_groups_ and active_executions_ are guarded by their respective mutexes.
  std::unordered_map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> active_groups_;
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
};

}  // namespace sobit_home

#endif  // SOBIT_HOME_LIBRARY__SOBIT_HOME_MOVEIT_SERVER_HPP_
