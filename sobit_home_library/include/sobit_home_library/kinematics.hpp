#ifndef SOBIT_HOME_LIBRARY_KINEMATICS_HPP
#define SOBIT_HOME_LIBRARY_KINEMATICS_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sobit_home
{

  class Kinematics
  {
  public:
    Kinematics();

    geometry_msgs::msg::Pose forward_kinematics(
        const std::vector<double> &joint_angles_rad,
        const bool is_right);

    std::vector<double> inverse_kinematics(
        const geometry_msgs::msg::TransformStamped &target_coord,
        const bool is_right);

    std::vector<double> look_at(
        const geometry_msgs::msg::TransformStamped &target_tf);

  private:
    static constexpr double BaseToShoulderDX = 0.17;
    static constexpr double BaseToShoulderDY = 0.20;
    static constexpr double LengthShoulderElbow = 0.40;
    static constexpr double LengthElbowWrist = 0.50;
    static constexpr double LengthHand = 0.18;
  };

}

#endif