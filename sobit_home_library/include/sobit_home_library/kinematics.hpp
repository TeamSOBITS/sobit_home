#ifndef SOBIT_HOME_LIBRARY_KINEMATICS_HPP
#define SOBIT_HOME_LIBRARY_KINEMATICS_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
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
        const geometry_msgs::msg::TransformStamped &lift_target_tf,
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

    // Analytical planar FK constants (derived from URDF, right arm, upper_roll=-π/2):
    //   EE_x = C0 + L1*cos(st + Phi0) + Lf*sin(st + el)
    //   EE_z =      L1*sin(st + Phi0) - Lf*cos(st + el)
    // where wrist_tilt = π/2 - st - el (keeps EE pointing +X).
    // Both arms are symmetric in XZ, so constants are arm-independent.
    static constexpr double IK_L1   =  0.39830861;  // shoulder pivot to elbow joint
    static constexpr double IK_Phi0 = -1.43239657;  // structural angle offset (rad)
    static constexpr double IK_C0   =  0.37448200;  // x-offset constant
    static constexpr double IK_Lf   =  0.55000000;  // elbow-to-EE forearm length

    // Z-seed model constants (used for initial guess only)
    static constexpr double ZA   = -0.0006;
    static constexpr double ZR   =  0.9465;
    static constexpr double ZPhi = -1.5117;

    // Workspace x bounds (IK-solver reachable, with wrist-forward constraint)
    // Probed minimum across all z: ~0.58 at z=+0.2. Use 0.56 as lower bound with margin.
    static constexpr double WS_X_MIN = 0.560;
    static constexpr double WS_X_MAX = 1.323;

    // Workspace z bounds (absolute, from FK sampling at joint limits)
    static constexpr double WS_Z_MIN = -0.944;
    static constexpr double WS_Z_MAX =  0.948;

    // Body lift joint range [m]
    static constexpr double LIFT_MIN = 0.0;
    static constexpr double LIFT_MAX = 0.69;

    // Forward kinematics (XZ only): returns (EE_x, EE_z) in body_lift_link.
    // wrist_tilt is implicitly π/2 - st - el.
    static void fk_xz(double st, double el, double &x_out, double &z_out);

    // One Newton-Raphson solve attempt from initial guess (st0, el0).
    // Returns true and fills (st_out, el_out) if converged to a valid solution.
    static bool newton_solve(double tx, double tz,
                             double st0, double el0,
                             double &st_out, double &el_out);

    // Multi-seed 2-DOF IK solver.
    static bool ik_solve(double tx, double tz, double &st_out, double &el_out);

    // Returns minimum base x-shift (dx) to bring (tx,tz) into IK workspace.
    // Positive dx = drive forward. Returns false if tz is out of z range.
    static bool base_shift_for_reachability(double tx, double tz, double &dx_out);
  };
}
#endif
