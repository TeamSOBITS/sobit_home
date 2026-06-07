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
    // Y-offset from body_lift_link origin to the right-arm shoulder pivot.
    static constexpr double BaseToShoulderDY = 0.205;

    // X-reach of the arm when upper_roll=-π/2 and shoulder_tilt=π/2 (arm straight forward).
    static constexpr double ArmReachX = 1.319;

    // Empirical EE_z model: EE_z = ZA + ZB*sin(st) + ZC*cos(st)
    // Fit from 9-point Gazebo sweep. Both arms use upper_roll=-π/2 and positive
    // shoulder_tilt. Constants are nearly identical for both arms.
    // ZR = sqrt(ZB²+ZC²), ZPhi = atan2(ZC,ZB)  →  EE_z = ZA + ZR*sin(st + ZPhi)
    // Invert: st = arcsin((tz - ZA) / ZR) - ZPhi
    // wrist_tilt = π/2 - st keeps hand_*_end_effector_link local X = body +X.
    static constexpr double ZA   = -0.0006;
    static constexpr double ZB   =  0.0559;
    static constexpr double ZC   = -0.9448;
    static constexpr double ZR   =  0.9465;
    static constexpr double ZPhi = -1.5117;
  };
}
#endif