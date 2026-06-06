#ifndef SOBIT_HOME_JOINT_ACTION_SERVER_HPP
#define SOBIT_HOME_JOINT_ACTION_SERVER_HPP

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urdf/model.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sobits_interfaces/action/move_joint.hpp"
#include "sobits_interfaces/action/move_to_pose.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_coord.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_tf.hpp"
#include "sobits_interfaces/srv/get_finger_angle.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{
  enum JointIds
  {
    ARM_R_SHOULDER_TILT,
    ARM_R_UPPER_ROLL,
    ARM_R_UPPER_FLEX,
    ARM_R_ELBOW,
    ARM_R_WRIST_TILT,
    ARM_R_WRIST_ROLL,

    HAND_R_FINGER_L_MCP,
    HAND_R_FINGER_L_PIP,
    HAND_R_FINGER_L_DIP,
    HAND_R_FINGER_C_MCP,
    HAND_R_FINGER_C_IP,
    HAND_R_FINGER_R_PIP,
    HAND_R_FINGER_R_DIP,

    ARM_L_SHOULDER_TILT,
    ARM_L_UPPER_ROLL,
    ARM_L_UPPER_FLEX,
    ARM_L_ELBOW,
    ARM_L_WRIST_TILT,
    ARM_L_WRIST_ROLL,

    HAND_L_FINGER_L_MCP,
    HAND_L_FINGER_L_PIP,
    HAND_L_FINGER_L_DIP,
    HAND_L_FINGER_C_MCP,
    HAND_L_FINGER_C_IP,
    HAND_L_FINGER_R_PIP,
    HAND_L_FINGER_R_DIP,

    BODY_LIFT,
    HEAD_PAN,
    HEAD_TILT,
    JOINT_NUM
  };

  struct PoseParams
  {
    std::string pose_name;

    double arm_right_shoulder_tilt;
    double arm_right_upper_roll;
    double arm_right_upper_flex;
    double arm_right_elbow;
    double arm_right_wrist_tilt;
    double arm_right_wrist_roll;

    double hand_right_finger_l_mcp;
    double hand_right_finger_l_pip;
    double hand_right_finger_l_dip;
    double hand_right_finger_c_mcp;
    double hand_right_finger_c_ip;
    double hand_right_finger_r_pip;
    double hand_right_finger_r_dip;

    double arm_left_shoulder_tilt;
    double arm_left_upper_roll;
    double arm_left_upper_flex;
    double arm_left_elbow;
    double arm_left_wrist_tilt;
    double arm_left_wrist_roll;

    double hand_left_finger_l_mcp;
    double hand_left_finger_l_pip;
    double hand_left_finger_l_dip;
    double hand_left_finger_c_mcp;
    double hand_left_finger_c_ip;
    double hand_left_finger_r_pip;
    double hand_left_finger_r_dip;

    double body_lift;
    double head_pan;
    double head_tilt;
  };

  class JointActionServer : public rclcpp::Node
  {
  public:
    using MoveJoint = sobits_interfaces::action::MoveJoint;
    using MoveToPose = sobits_interfaces::action::MoveToPose;

    using GetHandToTargetCoord = sobits_interfaces::srv::GetHandToTargetCoord;
    using GetHandToTargetTF = sobits_interfaces::srv::GetHandToTargetTF;
    using GetFingerAngle = sobits_interfaces::srv::GetFingerAngle;

    using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveJoint>;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveToPose>;

    explicit JointActionServer(const rclcpp::NodeOptions & options);
    ~JointActionServer();

    trajectory_msgs::msg::JointTrajectory set_joints(
      const std::vector<std::string> & target_joint_names,
      const std::vector<double> & target_joint_rad,
      const builtin_interfaces::msg::Duration & time_allowance,
      const std::string & group_name);

  private:
    const std::vector<std::string> JointNamesArmRight = {
        "arm_right_shoulder_tilt_joint",
        "arm_right_upper_roll_joint",
        "arm_right_upper_flex_joint",
        "arm_right_elbow_joint",
        "arm_right_wrist_tilt_joint",
        "arm_right_wrist_roll_joint"};

    const std::vector<std::string> JointNamesHandRight = {
        "hand_right_finger_l_mcp_joint",
        "hand_right_finger_l_pip_joint",
        "hand_right_finger_l_dip_joint",
        "hand_right_finger_c_mcp_joint",
        "hand_right_finger_c_ip_joint",
        "hand_right_finger_r_pip_joint",
        "hand_right_finger_r_dip_joint"};

    const std::vector<std::string> JointNamesArmLeft = {
        "arm_left_shoulder_tilt_joint",
        "arm_left_upper_roll_joint",
        "arm_left_upper_flex_joint",
        "arm_left_elbow_joint",
        "arm_left_wrist_tilt_joint",
        "arm_left_wrist_roll_joint"};

    const std::vector<std::string> JointNamesHandLeft = {
        "hand_left_finger_l_mcp_joint",
        "hand_left_finger_l_pip_joint",
        "hand_left_finger_l_dip_joint",
        "hand_left_finger_c_mcp_joint",
        "hand_left_finger_c_ip_joint",
        "hand_left_finger_r_pip_joint",
        "hand_left_finger_r_dip_joint"};

    const std::vector<std::string> JointNamesHead = {
        "head_pan_joint",
        "head_tilt_joint"};

    const std::vector<std::string> JointNamesBody = {
        "body_lift_joint"};

    std::vector<PoseParams> poses_;
    std::vector<PoseParams> right_hand_poses_;
    std::vector<PoseParams> left_hand_poses_;

    std::map<std::string, double> curt_joint_state_;
    std::unique_ptr<Kinematics> kinematics_;

    rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joints_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_to_pose_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_right_hand_to_pose_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_left_hand_to_pose_;

    rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_get_hand_to_coord_left_;
    rclcpp::Service<GetHandToTargetTF>::SharedPtr service_get_hand_to_tf_left_;
    rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_get_hand_to_coord_right_;
    rclcpp::Service<GetHandToTargetTF>::SharedPtr service_get_hand_to_tf_right_;
    rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_get_head_to_coord_;
    rclcpp::Service<GetHandToTargetTF>::SharedPtr service_get_head_to_tf_;
    rclcpp::Service<GetFingerAngle>::SharedPtr service_server_get_finger_angle_;

    void exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
    void exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
    void exe_move_right_hand_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
    void exe_move_left_hand_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

    void get_pos_to_coord(
      const std::shared_ptr<GetHandToTargetCoord::Request> request,
      std::shared_ptr<GetHandToTargetCoord::Response> response,
      bool is_right);
    void get_pos_to_tf(
      const std::shared_ptr<GetHandToTargetTF::Request> request,
      std::shared_ptr<GetHandToTargetTF::Response> response,
      bool is_right);
    void get_head_to_coord(
      const std::shared_ptr<GetHandToTargetCoord::Request> request,
      std::shared_ptr<GetHandToTargetCoord::Response> response);
    void get_head_to_tf(
      const std::shared_ptr<GetHandToTargetTF::Request> request,
      std::shared_ptr<GetHandToTargetTF::Response> response);
    void serve_get_finger_angle(
      const std::shared_ptr<GetFingerAngle::Request> request,
      std::shared_ptr<GetFingerAngle::Response> response);

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_arm_joint_control_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_right_arm_joint_control_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_hand_joint_control_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_right_hand_joint_control_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_body_joint_control_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_joint_control_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr urdf_timer_;
    bool urdf_loaded_{false};

    struct Limit
    {
      double lower, upper, velocity, effort;
      bool has{false};
    };
    std::unordered_map<std::string, Limit> joint_limits_;

    void load_joint_limits();
    bool parse_urdf_limits(const std::string & urdf_xml);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    std::string get_tf_frame(const std::string & frame_name);
  };

} // namespace sobit_home

#endif // SOBIT_HOME_JOINT_ACTION_SERVER_HPP