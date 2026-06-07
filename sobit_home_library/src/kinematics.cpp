#include "sobit_home_library/kinematics.hpp"

namespace sobit_home
{

  Kinematics::Kinematics() {}

  // -----------------------------------------------------------------------
  // Planar FK: EE position in body_lift_link frame for either arm.
  //
  // Derivation: with upper_roll=-π/2 locked, both shoulder_tilt and elbow
  // rotate around world -Y, keeping the arm in the X-Z plane.
  // Setting wrist_tilt = π/2 − st − el fixes the EE orientation to body +X.
  //
  //   EE_x = C0 + L1·cos(st + Phi0) + Lf·sin(st + el)
  //   EE_z =      L1·sin(st + Phi0) − Lf·cos(st + el)
  // -----------------------------------------------------------------------
  void Kinematics::fk_xz(double st, double el, double &x_out, double &z_out)
  {
    const double phi = st + el;
    x_out = IK_C0 + IK_L1 * std::cos(st + IK_Phi0) + IK_Lf * std::sin(phi);
    z_out =         IK_L1 * std::sin(st + IK_Phi0) - IK_Lf * std::cos(phi);
  }

  // -----------------------------------------------------------------------
  // One Newton-Raphson attempt from initial guess (st0, el0).
  // Analytical Jacobian eliminates finite-difference overhead.
  // -----------------------------------------------------------------------
  bool Kinematics::newton_solve(
      double tx, double tz,
      double st0, double el0,
      double &st_out, double &el_out)
  {
    constexpr double tol     = 1e-6;
    constexpr int    max_it  = 50;

    double st = st0, el = el0;

    for (int i = 0; i < max_it; ++i) {
      double x, z;
      fk_xz(st, el, x, z);

      const double ex = x - tx;
      const double ez = z - tz;
      if (std::abs(ex) < tol && std::abs(ez) < tol)
        break;

      const double phi = st + el;
      const double s1  = std::sin(st + IK_Phi0);
      const double c1  = std::cos(st + IK_Phi0);
      const double sp  = std::sin(phi);
      const double cp  = std::cos(phi);

      // Jacobian:
      // [dx/dst  dx/del]   [-L1·s1 + Lf·cp,  Lf·cp]
      // [dz/dst  dz/del] = [ L1·c1 + Lf·sp,  Lf·sp]
      const double J00 = -IK_L1 * s1 + IK_Lf * cp;
      const double J01 =               IK_Lf * cp;
      const double J10 =  IK_L1 * c1 + IK_Lf * sp;
      const double J11 =               IK_Lf * sp;

      const double det = J00 * J11 - J01 * J10;
      if (std::abs(det) < 1e-8)
        break;

      const double dst = -(J11 * ex - J01 * ez) / det;
      const double del = -(-J10 * ex + J00 * ez) / det;

      // Damped step: cap at 0.5 rad per iteration
      const double step = std::min(1.0, 0.5 / std::max({std::abs(dst), std::abs(del), 0.5}));
      st = std::clamp(st + step * dst, 0.0, M_PI);
      el = std::clamp(el + step * del, 0.0, 8.0 * M_PI / 9.0);

      // Enforce wrist_tilt limit ∈ [-π/2, π/2]: wt = π/2 - st - el ≥ -π/2 → el ≤ π - st
      if (M_PI_2 - st - el < -M_PI_2)
        el = M_PI - st;
    }

    double x_f, z_f;
    fk_xz(st, el, x_f, z_f);
    const double err = std::hypot(x_f - tx, z_f - tz);
    const double wt  = M_PI_2 - st - el;

    if (err > 1e-3 || std::abs(wt) > M_PI_2 + 0.01)
      return false;

    st_out = st;
    el_out = el;
    return true;
  }

  // -----------------------------------------------------------------------
  // Multi-seed 2-DOF IK solver.
  // Tries several initial guesses to avoid local-minima failures.
  // -----------------------------------------------------------------------
  bool Kinematics::ik_solve(double tx, double tz, double &st_out, double &el_out)
  {
    // Z-seed: initial shoulder_tilt from z-only model (el=0 approximation)
    const double arg0 = std::clamp((tz - ZA) / ZR, -1.0, 1.0);
    const double st_z = std::clamp(std::asin(arg0) - ZPhi, 0.05, M_PI - 0.05);

    // Seeds: (shoulder_tilt, elbow) initial guesses
    const double seeds[][2] = {
      {st_z,     0.3},
      {M_PI_2,   0.3},
      {M_PI / 4, 0.5},
      {3.0 * M_PI / 4, 0.3},
      {M_PI / 6, 0.2},
      {M_PI / 3, 1.0},
      {2.0 * M_PI / 3, 0.8},
      {0.1,      0.1},
      {M_PI - 0.1, 0.1},
    };

    for (const auto &seed : seeds) {
      double st_s, el_s;
      if (newton_solve(tx, tz, seed[0], seed[1], st_s, el_s))
        return st_out = st_s, el_out = el_s, true;
    }
    return false;
  }

  // -----------------------------------------------------------------------

  geometry_msgs::msg::Pose Kinematics::forward_kinematics(
      const std::vector<double> & /*joint_angles_rad*/,
      const geometry_msgs::msg::TransformStamped &base_target_tf,
      const bool is_right)
  {
    geometry_msgs::msg::Pose diff_pose;
    const double side_sign  = is_right ? 1.0 : -1.0;
    const double shoulder_y = side_sign * BaseToShoulderDY;

    const double tx = base_target_tf.transform.translation.x;
    const double ty = base_target_tf.transform.translation.y;

    const double diff_yaw       = std::atan2(ty - shoulder_y, tx);
    const double dist_to_target = std::hypot(tx, ty - shoulder_y);
    const double forward_dist   = dist_to_target - ArmReachX;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, diff_yaw);

    diff_pose.orientation = tf2::toMsg(q);
    diff_pose.position.x  = forward_dist;
    diff_pose.position.y  = 0.0;
    diff_pose.position.z  = base_target_tf.transform.translation.z;

    return diff_pose;
  }

  // -----------------------------------------------------------------------
  // Inverse kinematics for right or left arm.
  //
  // Solves for (shoulder_tilt, elbow) given target (x, z) in body_lift_link.
  // Both arms use the same XZ model (symmetric by arm symmetry).
  // wrist_tilt = π/2 − shoulder_tilt − elbow keeps EE pointing body +X.
  // upper_roll = −π/2 (locked), upper_flex = lower_flex = wrist_roll = 0.
  //
  // Returns [shoulder_tilt, upper_roll, upper_flex, elbow, lower_flex, wrist_tilt, wrist_roll]
  // or empty vector if target is unreachable.
  // -----------------------------------------------------------------------
  std::vector<double> Kinematics::inverse_kinematics(
      const geometry_msgs::msg::TransformStamped &lift_target_tf,
      const bool /*is_right*/)
  {
    const double tx = lift_target_tf.transform.translation.x;
    const double tz = lift_target_tf.transform.translation.z;

    double shoulder_tilt, elbow;
    if (!ik_solve(tx, tz, shoulder_tilt, elbow))
      return {};

    const double upper_roll = -M_PI_2;
    const double wrist_tilt = M_PI_2 - shoulder_tilt - elbow;

    return {shoulder_tilt, upper_roll, 0.0, elbow, 0.0, wrist_tilt, 0.0};
  }

  // -----------------------------------------------------------------------

  std::vector<double> Kinematics::look_at(
      const geometry_msgs::msg::TransformStamped &target_tf)
  {
    const double tx = target_tf.transform.translation.x;
    const double ty = target_tf.transform.translation.y;
    const double tz = target_tf.transform.translation.z;
    return {std::atan2(ty, tx), std::atan2(tz, std::hypot(tx, ty))};
  }

} // namespace sobit_home
