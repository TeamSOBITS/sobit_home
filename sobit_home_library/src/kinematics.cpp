#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{
    Kinematics::Kinematics() {}

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
        const double target_yaw)
    {
        (void)goal_coord;
        (void)is_right;
        (void)target_yaw;
        std::vector<double> joint_angles;
        return joint_angles;
    }

    std::vector<double> Kinematics::look_at(
        const geometry_msgs::msg::TransformStamped &target_tf)
    {
        double x = target_tf.transform.translation.x;
        double y = target_tf.transform.translation.y;
        double z = target_tf.transform.translation.z;

        double d = std::sqrt(x * x + y * y);

        double pan = std::atan2(y, x);
        double tilt = std::atan2(z, d);

        return {pan, tilt};
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