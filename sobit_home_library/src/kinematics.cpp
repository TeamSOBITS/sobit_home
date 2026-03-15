#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{
  Kinematics::Kinematics() {}

  geometry_msgs::msg::Pose Kinematics::forward_kinematics(
      const std::vector<double> &joint_angles_rad,
      const geometry_msgs::msg::TransformStamped &base_target_tf,
      const bool is_right)
  {
    geometry_msgs::msg::Pose diff_pose;
    const double side_sign = is_right ? 1.0 : -1.0;

    const double shoulder_y = is_right ? -BaseToShoulderDY : BaseToShoulderDY;
    const double arm_reach_x = LengthShoulderElbow * std::sin(joint_angles_rad.at(0) * side_sign)
                             + LengthElbowWrist * std::sin((joint_angles_rad.at(0) + joint_angles_rad.at(3)) * side_sign)
                             + LengthHand * std::sin((joint_angles_rad.at(0) + joint_angles_rad.at(3) + joint_angles_rad.at(4)) * side_sign);

    const double tx = base_target_tf.transform.translation.x;
    const double ty = base_target_tf.transform.translation.y;

    const double diff_yaw = std::atan2(ty - shoulder_y, tx);
    const double dist_shoulder_to_target = std::sqrt(tx * tx + std::pow(ty - shoulder_y, 2));
    const double forward_dist = dist_shoulder_to_target - arm_reach_x;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, diff_yaw);
    diff_pose.orientation = tf2::toMsg(q);

    diff_pose.position.x = forward_dist;
    diff_pose.position.y = 0.0;
    diff_pose.position.z = base_target_tf.transform.translation.z;

    return diff_pose;
  }

  std::vector<double> Kinematics::inverse_kinematics(
      const geometry_msgs::msg::TransformStamped &lift_target_tf,
      const bool is_right)
  {
    const double target_z = lift_target_tf.transform.translation.z;
    const double side_sign = is_right ? 1.0 : -1.0;
    const double upper_arm_roll = M_PI_2 * side_sign;

    double shoulder_tilt = 0.0;
    double elbow_flexion = 0.0;

    if (target_z > LengthElbowWrist)
    {
      return {};
    }
    else if (target_z >= 0.0)
    {
      shoulder_tilt = M_PI_2 * side_sign;
      elbow_flexion = std::asin(target_z / LengthElbowWrist) * side_sign;
    }
    else if (target_z >= -(LengthShoulderElbow + LengthElbowWrist))
    {
      const double L1 = LengthShoulderElbow;
      const double L2 = LengthElbowWrist;
      const double horiz_dist = LengthElbowWrist + LengthHand;
      const double vert_dist = target_z;

      const double dist_sq = horiz_dist * horiz_dist + vert_dist * vert_dist;
      const double dist = std::sqrt(dist_sq);

      const double cos_val_theta1 = (dist_sq + L1 * L1 - L2 * L2) / (2.0 * L1 * dist);

      const double theta1 = -std::acos(cos_val_theta1) + std::atan2(vert_dist, horiz_dist);
      const double theta2 = std::atan2(vert_dist - L1 * std::sin(theta1), horiz_dist - L1 * std::cos(theta1)) - theta1;

      shoulder_tilt = (theta1 + M_PI_2) * side_sign;
      elbow_flexion = theta2 * side_sign;
    }
    else
    {
      return {};
    }

    double wrist_tilt = -(shoulder_tilt + elbow_flexion) + (M_PI_2 * side_sign);

    return {shoulder_tilt, upper_arm_roll, 0.0, elbow_flexion, wrist_tilt, 0.0};
  }

  std::vector<double> Kinematics::look_at(
      const geometry_msgs::msg::TransformStamped &target_tf)
  {
    const double tx = target_tf.transform.translation.x;
    const double ty = target_tf.transform.translation.y;
    const double tz = target_tf.transform.translation.z;
    const double ground_dist = std::sqrt(tx * tx + ty * ty);

    return {std::atan2(ty, tx), std::atan2(tz, ground_dist)};
  }
}