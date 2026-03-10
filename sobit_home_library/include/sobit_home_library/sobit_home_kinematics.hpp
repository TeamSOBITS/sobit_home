#ifndef SOBIT_HOME_KINEMATICS_HPP
#define SOBIT_HOME_KINEMATICS_HPP

#include <vector>
#include <string>
#include <cmath>
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
        const bool is_right,
        const double target_yaw);

    std::vector<double> inverse_kinematics(
        const geometry_msgs::msg::TransformStamped &goal_coord,
        const bool is_right,
        bool is_one_link,
        const double target_yaw);

    std::vector<double> look_at(
        const geometry_msgs::msg::TransformStamped &target_tf);

    static geometry_msgs::msg::Vector3 get_euler_from_quat(const geometry_msgs::msg::Quaternion &quat);
    static geometry_msgs::msg::Quaternion get_quat_from_euler(const geometry_msgs::msg::Vector3 &rpy);

  private:
    static constexpr double BaseToShoulderDX = 0.11275;
    static constexpr double BaseToShoulderDY = 0.2775;
    static constexpr double BaseToShoulderDZ = 0.936058;
    static constexpr double LengthShoulderElbow = 0.399949;
    static constexpr double LengthElbowWrist = 0.5000003;
    static constexpr double LengthHand = 0.1811;
    static constexpr double BodylinkToHeadtiltDZ = 0.216;
  };

}

#endif