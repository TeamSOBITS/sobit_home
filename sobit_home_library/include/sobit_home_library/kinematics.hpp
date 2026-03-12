#ifndef SOBIT_HOME_LIBRARY_KINEMATICS_HPP
#define SOBIT_HOME_LIBRARY_KINEMATICS_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sobit_home
{

class Kinematics
{
public:
    Kinematics();

    geometry_msgs::msg::TransformStamped forward_kinematics(
        const std::vector<double> &target_joint_rad,
        const bool is_right);

    std::vector<double> inverse_kinematics(
        const geometry_msgs::msg::TransformStamped &goal_coord,
        const bool is_right);

    std::vector<double> look_at(
        const geometry_msgs::msg::TransformStamped &target_tf);

    static geometry_msgs::msg::Vector3 get_euler_from_quat(const geometry_msgs::msg::Quaternion &quat);
    static geometry_msgs::msg::Quaternion get_quat_from_euler(const geometry_msgs::msg::Vector3 &rpy);

private:
    static constexpr double BaseToShoulderDY = 0.28;
    static constexpr double LengthShoulderElbow = 0.40;
    static constexpr double LengthElbowWrist = 0.50;
    static constexpr double LengthHand = 0.18;
};

}

#endif