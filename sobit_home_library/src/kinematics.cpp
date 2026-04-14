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
    const double shoulder_y = side_sign * -BaseToShoulderDY;

    const double s_tilt = joint_angles_rad.at(0) * side_sign;
    const double e_flex = joint_angles_rad.at(3) * side_sign;
    const double w_tilt = joint_angles_rad.at(4);

    const double arm_reach_x = LengthShoulderElbow * std::sin(s_tilt)
                            + LengthElbowWrist    * std::sin(s_tilt + e_flex)
                            + LengthHand          * std::sin(s_tilt + e_flex + w_tilt);

    const double tx = base_target_tf.transform.translation.x;
    const double ty = base_target_tf.transform.translation.y;

    const double diff_yaw = std::atan2(ty - shoulder_y, tx);
    const double dist_to_target = std::hypot(tx, ty - shoulder_y);
    const double forward_dist = dist_to_target - arm_reach_x;

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

    if (target_z > LengthElbowWrist || target_z < -(LengthShoulderElbow + LengthElbowWrist))
    {
      return {};
    }

    double s_tilt = 0.0;
    double e_flex = 0.0;

    if (target_z >= 0.0)
    {
      s_tilt = M_PI_2 * side_sign;
      e_flex = std::asin(target_z / LengthElbowWrist) * side_sign;
    }
    else
    {
      const double L1 = LengthShoulderElbow;
      const double L2 = LengthElbowWrist;
      const double horiz_dist = LengthElbowWrist + LengthHand;
      const double vert_dist = target_z;

      const double dist_sq = std::pow(horiz_dist, 2) + std::pow(vert_dist, 2);
      const double dist = std::sqrt(dist_sq);

      const double cos_v = (dist_sq + std::pow(L1, 2) - std::pow(L2, 2)) / (2.0 * L1 * dist);
      const double theta1 = -std::acos(std::clamp(cos_v, -1.0, 1.0)) + std::atan2(vert_dist, horiz_dist);
      const double theta2 = std::atan2(vert_dist - L1 * std::sin(theta1), horiz_dist - L1 * std::cos(theta1)) - theta1;

      s_tilt = (theta1 + M_PI_2) * side_sign;
      e_flex = theta2 * side_sign;
    }

    const double w_tilt = -(s_tilt + e_flex) + (M_PI_2 * side_sign);

    return {s_tilt, upper_arm_roll, 0.0, e_flex, w_tilt, 0.0};
  }

  std::vector<double> Kinematics::look_at(
      const geometry_msgs::msg::TransformStamped &target_tf)
  {

    const double tx = target_tf.transform.translation.x;
    const double ty = target_tf.transform.translation.y;
    const double tz = target_tf.transform.translation.z;

    const double ground_dist = std::hypot(tx, ty);

    return {std::atan2(ty, tx), std::atan2(tz, ground_dist)};
  }
}