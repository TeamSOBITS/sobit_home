#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{
  Kinematics::Kinematics() {}

  geometry_msgs::msg::Pose Kinematics::forward_kinematics(
      const std::vector<double> &joint_angles_rad,
      const geometry_msgs::msg::TransformStamped &base_target_tf,
      const bool is_right)
  {
    geometry_msgs::msg::Pose hand_pose;

    double s_tilt = joint_angles_rad.at(0);
    double e_flex = joint_angles_rad.at(3);
    double dy = is_right ? -BaseToShoulderDY : BaseToShoulderDY;

    double forearm_total = LengthElbowWrist + LengthHand;

    hand_pose.position.x = base_target_tf.transform.translation.x;
    hand_pose.position.y = dy + LengthShoulderElbow * std::cos(s_tilt) + forearm_total * std::cos(s_tilt + e_flex);
    hand_pose.position.z = base_target_tf.transform.translation.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    hand_pose.orientation = tf2::toMsg(q);

    return hand_pose;
  }

  std::vector<double> Kinematics::inverse_kinematics(
      const geometry_msgs::msg::TransformStamped &lift_target_tf,
      const bool is_right)
  {
    const double target_z = lift_target_tf.transform.translation.z;
    const double sign = is_right ? 1.0 : -1.0;
    const double upper_arm_roll = M_PI_2 * sign;

    double shoulder_tilt = 0.0;
    double elbow_flexion = 0.0;

    if (target_z > LengthElbowWrist)
    {
      return {};
    }
    else if (target_z >= 0.0)
    {
      shoulder_tilt = M_PI_2 * sign;
      elbow_flexion = std::asin(target_z / LengthElbowWrist) * sign;
    }
    else if (target_z >= -(LengthShoulderElbow + LengthElbowWrist))
    {
      const double L1 = LengthShoulderElbow;
      const double L2 = LengthElbowWrist;
      const double x = LengthElbowWrist + LengthHand;
      const double z = target_z;

      const double dist_sq = x * x + z * z;
      const double dist = std::sqrt(dist_sq);

      const double cos_val_theta1 = (dist_sq + L1 * L1 - L2 * L2) / (2.0 * L1 * dist);

      const double theta1 = -std::acos(cos_val_theta1) + std::atan2(z, x);
      const double theta2 = std::atan2(z - L1 * std::sin(theta1), x - L1 * std::cos(theta1)) - theta1;

      shoulder_tilt = (theta1 + M_PI_2) * sign;
      elbow_flexion = theta2 * sign;
    }
    else
    {
      return {};
    }

    double wrist_tilt = -(shoulder_tilt + elbow_flexion) + (M_PI_2 * sign);

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