#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{

  Kinematics::Kinematics()
  {
  }

  geometry_msgs::msg::TransformStamped Kinematics::forward_kinematics(
      const std::vector<double> &target_joint_rad,
      const bool is_right,
      const double target_yaw)
  {
    (void)target_joint_rad;
    (void)is_right;
    (void)target_yaw;
    geometry_msgs::msg::TransformStamped tf;
    return tf;
  }

  std::vector<double> Kinematics::inverse_kinematics(
      const geometry_msgs::msg::TransformStamped &goal_coord,
      const bool is_right,
      bool is_one_link,
      const double target_yaw)
  {
    (void)goal_coord;
    (void)is_right;
    (void)is_one_link;
    (void)target_yaw;
    std::vector<double> target_joint_rad(5, 0.0);
    return target_joint_rad;
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