#include <map>

#include "sobits_interfaces/action/move_joint.hpp"
#include "sobits_interfaces/action/move_to_pose.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_coord.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_tf.hpp"
#include "sobits_interfaces/srv/move_hand_to_target_coord.hpp"
#include "sobits_interfaces/srv/move_hand_to_target_tf.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/point.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>


namespace sobit_home
{

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
  double hand_right_finger_l_dip;
  double hand_right_finger_l_pip;
  double hand_right_finger_c_mcp;
  double hand_right_finger_c_ip;
  double hand_right_finger_r_dip;
  double hand_right_finger_r_pip;
  double arm_left_shoulder_tilt;
  double arm_left_upper_roll;
  double arm_left_upper_flex;
  double arm_left_elbow;
  double arm_left_wrist_tilt;
  double arm_left_wrist_roll;
  double hand_left_finger_l_mcp;
  double hand_left_finger_l_dip;
  double hand_left_finger_l_pip;
  double hand_left_finger_c_mcp;
  double hand_left_finger_c_ip;
  double hand_left_finger_r_dip;
  double hand_left_finger_r_pip;
  double body_lift;
  double head_pan;
  double head_tilt;
};

enum JointIds
{
  Arm_R_Shoulder_Tilt_Joint = 0,
  Arm_R_Upper_Roll_Joint,
  Arm_R_Upper_Flex_Joint,
  Arm_R_Elbow_Joint,
  Arm_R_Wrist_Tilt_Joint,
  Arm_R_Wrist_Roll_Joint,
  Hand_R_Finger_L_Mcp_Joint,
  Hand_R_Finger_L_Dip_Joint,
  Hand_R_Finger_L_Pip_Joint,
  Hand_R_Finger_C_Mcp_Joint,
  Hand_R_Finger_C_Ip_Joint,
  Hand_Right_Finger_R_Dip_Joint,
  Hand_right_finger_r_pip_joint,
  Arm_L_Shoulder_Tilt_Joint,
  Arm_L_Upper_Roll_Joint,
  Arm_L_Upper_Flex_Joint,
  Arm_L_Elbow_Joint,
  Arm_L_Wrist_Tilt_Joint,
  Arm_L_Wrist_Roll_Joint,
  Hand_L_Finger_L_Mcp_Joint,
  Hand_L_Finger_L_Dip_Joint,
  Hand_L_Finger_L_Pip_Joint,
  Hand_L_Finger_C_Mcp_Joint,
  Hand_L_Finger_C_Ip_Joint,
  Hand_L_Finger_R_Dip_Joint,
  Hand_L_Finger_R_Pip_Joint,
  Body_Lift_Joint,
  Head_Pan_Joint,
  Head_Tilt_Joint,
  JointNum
};

class JointActionServer : public rclcpp::Node
{
public:
  using MoveJoint = sobits_interfaces::action::MoveJoint;
  using MoveToPose = sobits_interfaces::action::MoveToPose;
  // using MoveHandToTargetCoord = sobits_interfaces::action::MoveHandToTargetCoord;
  // using MoveHandToTargetTF = sobits_interfaces::action::MoveHandToTargetTF;
  using MoveHandToTargetCoord = sobits_interfaces::srv::MoveHandToTargetCoord;
  using MoveHandToTargetTF = sobits_interfaces::srv::MoveHandToTargetTF;

  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveJoint>;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveToPose>;
  // using GoalHandleMoveHandToCoord = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetCoord>;
  // using GoalHandleMoveHandToTf = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetTF>;


  explicit JointActionServer(const rclcpp::NodeOptions & options);
  ~JointActionServer();

  geometry_msgs::msg::Vector3 get_euler_from_quat(
    const geometry_msgs::msg::Quaternion& quat);
  geometry_msgs::msg::Quaternion get_quat_from_euler(
    const geometry_msgs::msg::Vector3& rpy);
  geometry_msgs::msg::TransformStamped forward_kinematics(
    const std::vector<double> &target_joint_rad,
    const bool is_right,
    const double target_yaw);  // target_yaw should be eliminated in the future.
  std::vector<double> inverse_kinematics(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    const bool is_right, bool is_one_rink,
    const double target_yaw);  // target_yaw should be eliminated in the future.
  std::vector<trajectory_msgs::msg::JointTrajectory> set_joints(
    const std::vector<std::string> &target_joint_names,
    const std::vector<double> &target_joint_rad,
    const builtin_interfaces::msg::Duration &time_allowance);

private:
  const std::vector<std::string> JointNames = {
    "arm_right_shoulder_tilt_joint",
    "arm_right_upper_roll_joint",
    "arm_right_upper_flex_joint",
    "arm_right_elbow_joint",
    "arm_right_wrist_tilt_joint",
    "arm_right_wrist_roll_joint",
    "hand_right_finger_l_mcp_joint",
    "hand_right_finger_l_dip_joint",
    "hand_right_finger_l_pip_joint",
    "hand_right_finger_c_mcp_joint",
    "hand_right_finger_c_ip_joint",
    "hand_right_finger_r_dip_joint",
    "hand_right_finger_r_pip_joint",
    "arm_left_shoulder_tilt_joint",
    "arm_left_upper_roll_joint",
    "arm_left_upper_flex_joint",
    "arm_left_elbow_joint",
    "arm_left_wrist_tilt_joint",
    "arm_left_wrist_roll_joint",
    "hand_left_finger_l_mcp_joint",
    "hand_left_finger_l_dip_joint",
    "hand_left_finger_l_pip_joint",
    "hand_left_finger_c_mcp_joint",
    "hand_left_finger_c_ip_joint",
    "hand_left_finger_r_dip_joint",
    "hand_left_finger_r_pip_joint",
    "body_lift_joint",
    "head_pan_joint",
    "head_tilt_joint"
  };

  const std::vector<std::string> JointNamesArmRight = {
    "arm_right_shoulder_tilt_joint",
    "arm_right_upper_roll_joint",
    "arm_right_upper_flex_joint",
    "arm_right_elbow_joint",
    "arm_right_wrist_tilt_joint",
    "arm_right_wrist_roll_joint"
  };
  
  const std::vector<std::string> JointNamesHandRight = {
    "hand_right_finger_l_mcp_joint",
    "hand_right_finger_l_dip_joint",
    "hand_right_finger_l_pip_joint",
    "hand_right_finger_c_mcp_joint",
    "hand_right_finger_c_ip_joint",
    "hand_right_finger_r_dip_joint",
    "hand_right_finger_r_pip_joint"
  };

  const std::vector<std::string> JointNamesArmLeft = {
    "arm_left_shoulder_tilt_joint",
    "arm_left_upper_roll_joint",
    "arm_left_upper_flex_joint",
    "arm_left_elbow_joint",
    "arm_left_wrist_tilt_joint",
    "arm_left_wrist_roll_joint"
  };

  const std::vector<std::string> JointNamesHandLeft = {
    "hand_left_finger_l_mcp_joint",
    "hand_left_finger_l_dip_joint",
    "hand_left_finger_l_pip_joint",
    "hand_left_finger_c_mcp_joint",
    "hand_left_finger_c_ip_joint",
    "hand_left_finger_r_dip_joint",
    "hand_left_finger_r_pip_joint"
  };

  const std::vector<std::string> JointNamesHead = {
    "head_pan_joint",
    "head_tilt_joint"
  };

  const std::vector<std::string> JointNamesBody = {
    "body_lift_joint"
  };

  static constexpr double BaseToShoulderDX    = 0.11275;
  static constexpr double BaseToShoulderDY    = 0.2775;  
  static constexpr double BaseToShoulderDZ    = 0.936058; 
  static constexpr double LengthShoulderElbow = 0.399949;
  static constexpr double LengthElbowWrist    = 0.5000003;
  static constexpr double LengthHand          = 0.1811;   
  /*box高さ 0.291m*/

  std::vector<PoseParams> poses_;
  std::map<std::string, double> init_joint_state_;
  std::map<std::string, double> curt_joint_state_;

  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joints_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_to_pose_;
  rclcpp::Service<MoveHandToTargetCoord>::SharedPtr service_server_move_hand_to_coord_left_;
  rclcpp::Service<MoveHandToTargetTF>::SharedPtr service_server_move_hand_to_tf_left_;
  rclcpp::Service<MoveHandToTargetCoord>::SharedPtr service_server_move_hand_to_coord_right_;
  rclcpp::Service<MoveHandToTargetTF>::SharedPtr service_server_move_hand_to_tf_right_;
  rclcpp::Service<MoveHandToTargetCoord>::SharedPtr service_server_move_hand_to_coord_one_left_;
  rclcpp::Service<MoveHandToTargetTF>::SharedPtr service_server_move_hand_to_tf_one_left_;
  rclcpp::Service<MoveHandToTargetCoord>::SharedPtr service_server_move_hand_to_coord_one_right_;
  rclcpp::Service<MoveHandToTargetTF>::SharedPtr service_server_move_hand_to_tf_one_right_;

  rclcpp_action::GoalResponse handle_move_joints_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveJoint::Goal> goal);
  rclcpp_action::GoalResponse handle_move_to_pose_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_move_joints_cancel(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  rclcpp_action::CancelResponse handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void handle_move_joints_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void serve_move_hand_to_coord(const std::shared_ptr<MoveHandToTargetCoord::Request> request, std::shared_ptr<MoveHandToTargetCoord::Response> response, bool is_right, bool is_one_rink);
  void serve_move_hand_to_tf(const std::shared_ptr<MoveHandToTargetTF::Request> request, std::shared_ptr<MoveHandToTargetTF::Response> response, bool is_right, bool is_one_rink);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_arm_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_right_arm_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_hand_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_right_hand_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_body_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_joint_control_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
}; // class JointActionServer

inline geometry_msgs::msg::Vector3 JointActionServer::get_euler_from_quat(
  const geometry_msgs::msg::Quaternion& msg_quat)
{
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Vector3 euler;

  tf2::fromMsg(msg_quat, tf_quat);
  tf_quat.normalize();
  tf2::Matrix3x3(tf_quat).getRPY(euler.x, euler.y, euler.z);

  return euler;  
}

inline geometry_msgs::msg::Quaternion JointActionServer::get_quat_from_euler(
  const geometry_msgs::msg::Vector3& euler)
{
  tf2::Quaternion tf_quat;

  tf_quat.setRPY(euler.x, euler.y, euler.z);

  return tf2::toMsg(tf_quat);
}

} // namespace sobit_home

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_home::JointActionServer)
