#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{

  Kinematics::Kinematics() {}

  // Returns the wheel-drive pose (forward distance + yaw) needed for the mobile base.
  // joint_angles_rad layout: [shoulder_tilt, upper_roll, upper_flex, elbow,
  //                            lower_flex, wrist_tilt, wrist_roll]
  geometry_msgs::msg::Pose Kinematics::forward_kinematics(
      const std::vector<double> & /*joint_angles_rad*/,
      const geometry_msgs::msg::TransformStamped &base_target_tf,
      const bool is_right)
  {
    geometry_msgs::msg::Pose diff_pose;
    const double side_sign  = is_right ? 1.0 : -1.0;
    const double shoulder_y = side_sign * BaseToShoulderDY;

    const double tx = base_target_tf.transform.translation.x;
    const double ty = base_target_tf.transform.translation.y;

    // Yaw robot to face the target, then drive until arm reach aligns.
    const double diff_yaw       = std::atan2(ty - shoulder_y, tx);
    const double dist_to_target = std::hypot(tx, ty - shoulder_y);
    const double forward_dist   = dist_to_target - ArmReachX;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, diff_yaw);

    diff_pose.orientation = tf2::toMsg(q);
    diff_pose.position.x  = forward_dist;
    diff_pose.position.y  = 0.0;
    diff_pose.position.z  = base_target_tf.transform.translation.z;

    return diff_pose;
  }

  // Inverse kinematics for right or left arm.
  //
  // Both arms use upper_roll=-π/2 and positive shoulder_tilt to sweep the EE in
  // the body X-Z plane (y fixed at ±0.305). The EE_z model is identical for both arms:
  //   EE_z = ZA + ZB*sin(st) + ZC*cos(st)
  // Invert: st = arcsin((tz - ZA) / ZR) - ZPhi
  // wrist_tilt = π/2 - st keeps hand_*_end_effector_link local X = body +X.
  //
  // Returns [shoulder_tilt, upper_roll, upper_flex, elbow, lower_flex, wrist_tilt, wrist_roll]
  std::vector<double> Kinematics::inverse_kinematics(
      const geometry_msgs::msg::TransformStamped &lift_target_tf,
      const bool /*is_right*/)
  {
    const double tz = lift_target_tf.transform.translation.z;

    const double arg = (tz - ZA) / ZR;
    if (std::abs(arg) > 1.0)
      return {};

    const double shoulder_tilt = std::asin(arg) - ZPhi;   // always positive
    const double upper_roll    = -M_PI_2;                  // same for both arms
    const double wrist_tilt    = M_PI_2 - shoulder_tilt;  // keeps EE local X = body +X

    return {shoulder_tilt, upper_roll, 0.0, 0.0, 0.0, wrist_tilt, 0.0};
  }

  std::vector<double> Kinematics::look_at(
      const geometry_msgs::msg::TransformStamped &target_tf)
  {
    const double tx = target_tf.transform.translation.x;
    const double ty = target_tf.transform.translation.y;
    const double tz = target_tf.transform.translation.z;
    return {std::atan2(ty, tx), std::atan2(tz, std::hypot(tx, ty))};
  }

} // namespace sobit_home
