#include "sobit_home_library/sobit_home_kinematics.hpp"

namespace sobit_home
{

  Kinematics::Kinematics() {}

  geometry_msgs::msg::TransformStamped Kinematics::forward_kinematics(
      const std::vector<double> &target_joint_rad,
      const bool is_right,
      const double target_yaw)
  {

    geometry_msgs::msg::TransformStamped tf;
    double l1 = LengthShoulderElbow;
    double l2 = LengthElbowWrist;
    double l3 = LengthHand;

    double q1 = target_joint_rad[1];
    double q2 = target_joint_rad[2];
    double q3 = target_joint_rad[3];

    double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2) + l3 * std::cos(q1 + q2 + q3);
    double z = l1 * std::sin(q1) + l2 * std::sin(q1 + q2) + l3 * std::sin(q1 + q2 + q3);

    tf.transform.translation.x = BaseToShoulderDX + x * std::cos(target_yaw);
    tf.transform.translation.y = (is_right ? -1.0 : 1.0) * BaseToShoulderDY + x * std::sin(target_yaw);
    tf.transform.translation.z = BaseToShoulderDZ + z;

    return tf;
  }

  std::vector<double> Kinematics::inverse_kinematics(
      const geometry_msgs::msg::TransformStamped &goal_coord,
      const bool is_right,
      bool is_one_link,
      const double target_yaw)
  {

    std::vector<double> target_joint_rad;

    double gx = goal_coord.transform.translation.x - BaseToShoulderDX;
    double gy = goal_coord.transform.translation.y - (is_right ? -1.0 : 1.0) * BaseToShoulderDY;
    double gz = goal_coord.transform.translation.z - BaseToShoulderDZ;

    double x = std::sqrt(gx * gx + gy * gy);
    double z = gz;

    double l1 = LengthShoulderElbow;
    double l2 = LengthElbowWrist + LengthHand;

    double d2 = x * x + z * z;
    double cos_q2 = (d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);

    if (std::abs(cos_q2) > 1.0)
      return {};

    double q2 = std::acos(cos_q2);
    double q1 = std::atan2(z, x) - std::atan2(l2 * std::sin(q2), l1 + l2 * std::cos(q2));

    target_joint_rad.push_back(0.0);
    target_joint_rad.push_back(q1);
    target_joint_rad.push_back(q2);
    target_joint_rad.push_back(0.0);

    return target_joint_rad;
  }

  std::vector<double> Kinematics::look_at(
      const geometry_msgs::msg::TransformStamped &target_tf)
  {

    std::vector<double> head_joint_rad;

    double dx = target_tf.transform.translation.x;
    double dy = target_tf.transform.translation.y;
    double dz = target_tf.transform.translation.z - (BaseToShoulderDZ + BodylinkToHeadtiltDZ);

    double pan = std::atan2(dy, dx);
    double tilt = std::atan2(-dz, std::sqrt(dx * dx + dy * dy));

    head_joint_rad.push_back(pan);
    head_joint_rad.push_back(tilt);

    return head_joint_rad;
  }

  geometry_msgs::msg::Vector3 Kinematics::get_euler_from_quat(const geometry_msgs::msg::Quaternion &quat)
  {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = r;
    rpy.y = p;
    rpy.z = y;
    return rpy;
  }

  geometry_msgs::msg::Quaternion Kinematics::get_quat_from_euler(const geometry_msgs::msg::Vector3 &rpy)
  {
    tf2::Quaternion q;
    q.setRPY(rpy.x, rpy.y, rpy.z);
    return tf2::toMsg(q);
  }

}