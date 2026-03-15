#ifndef SOBIT_HOME_LIBRARY_KINEMATICS_HPP
#define SOBIT_HOME_LIBRARY_KINEMATICS_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sobit_home
{
  class Kinematics
  {
  public:
    Kinematics();

    geometry_msgs::msg::Pose forward_kinematics(
        const std::vector<double> &joint_angles_rad,
        const geometry_msgs::msg::TransformStamped &base_target_tf,
        const bool is_right);

    std::vector<double> inverse_kinematics(
        const geometry_msgs::msg::TransformStamped &lift_target_tf,
        const bool is_right);

    std::vector<double> look_at(const geometry_msgs::msg::TransformStamped &target_tf);

  private:
    static constexpr double BaseToShoulderDY = 0.278;
    static constexpr double LengthShoulderElbow = 0.398;
    static constexpr double LengthElbowWrist = 0.566;
    static constexpr double LengthHand = 0.18;
  };
}
#endif