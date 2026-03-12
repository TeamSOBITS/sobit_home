#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{
  Kinematics::Kinematics() {}

  geometry_msgs::msg::Pose Kinematics::forward_kinematics(
      const std::vector<double> &joint_angles_rad,
      const bool is_right)
  {
    geometry_msgs::msg::Pose hand_pose;
    if (joint_angles_rad.size() < 6)
      return hand_pose;

    double shoulder_tilt = joint_angles_rad[0];
    double elbow_flexion = joint_angles_rad[3];

    if (!is_right)
    {
      shoulder_tilt = -shoulder_tilt;
      elbow_flexion = -elbow_flexion;
    }

    double total_pitch_angle = shoulder_tilt + elbow_flexion;
    double forearm_length = LengthElbowWrist + LengthHand;

    double x_rel = LengthShoulderElbow * std::cos(shoulder_tilt) + forearm_length * std::cos(total_pitch_angle);
    double z_rel = LengthShoulderElbow * std::sin(shoulder_tilt) + forearm_length * std::sin(total_pitch_angle);

    hand_pose.position.x = BaseToShoulderDX + x_rel;
    hand_pose.position.y = is_right ? -BaseToShoulderDY : BaseToShoulderDY;
    hand_pose.position.z = z_rel;

    tf2::Quaternion orientation;
    orientation.setRPY(0, -total_pitch_angle, 0);
    hand_pose.orientation = tf2::toMsg(orientation);

    return hand_pose;
  }

  std::vector<double> Kinematics::inverse_kinematics(
      const geometry_msgs::msg::TransformStamped &target_coord,
      const bool is_right)
  {
    const double target_z = target_coord.transform.translation.z;
    const double forearm_length = LengthElbowWrist + LengthHand;
    const double upper_arm_roll = is_right ? 1.57 : -1.57;

    double shoulder_tilt = 0.0;
    double elbow_flexion = 0.0;

    if (target_z > forearm_length)
    {
      return {};
    }
    else if (target_z >= 0.0)
    {
      shoulder_tilt = is_right ? 1.57 : -1.57;
      double normalized_z = std::clamp(target_z / forearm_length, -1.0, 1.0);
      double angle_rad = std::asin(normalized_z);
      elbow_flexion = is_right ? std::clamp(angle_rad, 0.0, 1.57) : std::clamp(-angle_rad, -1.57, 0.0);
    }
    else if (target_z >= -(LengthShoulderElbow + forearm_length))
    {
      const double reach_x = forearm_length;
      const double dist_sq = reach_x * reach_x + target_z * target_z;
      const double dist = std::sqrt(dist_sq);

      double cos_elbow = (dist_sq - std::pow(LengthShoulderElbow, 2) - std::pow(forearm_length, 2)) / (2.0 * LengthShoulderElbow * forearm_length);
      double elbow_val = std::acos(std::clamp(cos_elbow, -1.0, 1.0));

      double cos_shoulder = (std::pow(LengthShoulderElbow, 2) + dist_sq - std::pow(forearm_length, 2)) / (2.0 * LengthShoulderElbow * dist);
      double shoulder_base = std::atan2(target_z, reach_x) + std::acos(std::clamp(cos_shoulder, -1.0, 1.0));

      if (is_right)
      {
        shoulder_tilt = std::clamp(shoulder_base, 0.0, 1.57);
        elbow_flexion = std::clamp(elbow_val, 0.0, 1.57);
      }
      else
      {
        shoulder_tilt = std::clamp(-shoulder_base, -1.57, 0.0);
        elbow_flexion = std::clamp(-elbow_val, -1.57, 0.0);
      }
    }
    else
    {
      return {};
    }

    double wrist_tilt = -(shoulder_tilt + elbow_flexion) + (is_right ? 1.57 : -1.57);

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