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
void Kinematics::fk_xz(double st, double el, double & x_out, double & z_out)
{
  const double phi = st + el;
  x_out = IK_C0 + IK_L1 * std::cos(st + IK_Phi0) + IK_Lf * std::sin(phi);
  z_out = IK_L1 * std::sin(st + IK_Phi0) - IK_Lf * std::cos(phi);
}

  // -----------------------------------------------------------------------
  // One Newton-Raphson attempt from initial guess (st0, el0).
  // Analytical Jacobian eliminates finite-difference overhead.
  // -----------------------------------------------------------------------
bool Kinematics::newton_solve(
  double tx, double tz,
  double st0, double el0,
  double & st_out, double & el_out)
{
  constexpr double tol = 1e-6;
  constexpr int    max_it = 50;

  double st = st0, el = el0;

  for (int i = 0; i < max_it; ++i) {
    double x, z;
    fk_xz(st, el, x, z);

    const double ex = x - tx;
    const double ez = z - tz;
    if (std::abs(ex) < tol && std::abs(ez) < tol) {
      break;
    }

    const double phi = st + el;
    const double s1 = std::sin(st + IK_Phi0);
    const double c1 = std::cos(st + IK_Phi0);
    const double sp = std::sin(phi);
    const double cp = std::cos(phi);

      // Jacobian:
      // [dx/dst  dx/del]   [-L1·s1 + Lf·cp,  Lf·cp]
      // [dz/dst  dz/del] = [ L1·c1 + Lf·sp,  Lf·sp]
    const double J00 = -IK_L1 * s1 + IK_Lf * cp;
    const double J01 = IK_Lf * cp;
    const double J10 = IK_L1 * c1 + IK_Lf * sp;
    const double J11 = IK_Lf * sp;

    const double det = J00 * J11 - J01 * J10;
    if (std::abs(det) < 1e-8) {
      break;
    }

    const double dst = -(J11 * ex - J01 * ez) / det;
    const double del = -(-J10 * ex + J00 * ez) / det;

      // Damped step: cap at 0.5 rad per iteration
    const double step = std::min(1.0, 0.5 / std::max({std::abs(dst), std::abs(del), 0.5}));
    st = std::clamp(st + step * dst, 0.0, M_PI);
    el = std::clamp(el + step * del, 0.0, 8.0 * M_PI / 9.0);

      // Enforce wrist_tilt limit ∈ [-π/2, π/2]: wt = π/2 - st - el ≥ -π/2 → el ≤ π - st
    if (M_PI_2 - st - el < -M_PI_2) {
      el = M_PI - st;
    }
  }

  double x_f, z_f;
  fk_xz(st, el, x_f, z_f);
  const double err = std::hypot(x_f - tx, z_f - tz);
  const double wt = M_PI_2 - st - el;

  if (err > 1e-3 || std::abs(wt) > M_PI_2 + 0.01) {
    return false;
  }

  st_out = st;
  el_out = el;
  return true;
}

  // -----------------------------------------------------------------------
  // Multi-seed 2-DOF IK solver.
  // Tries several initial guesses to avoid local-minima failures.
  // -----------------------------------------------------------------------
bool Kinematics::ik_solve(double tx, double tz, double & st_out, double & el_out)
{
    // Z-seed: initial shoulder_tilt from z-only model (el=0 approximation)
  const double arg0 = std::clamp((tz - ZA) / ZR, -1.0, 1.0);
  const double st_z = std::clamp(std::asin(arg0) - ZPhi, 0.05, M_PI - 0.05);

    // Seeds: (shoulder_tilt, elbow) initial guesses
  const double seeds[][2] = {
    {st_z, 0.3},
    {M_PI_2, 0.3},
    {M_PI / 4, 0.5},
    {3.0 * M_PI / 4, 0.3},
    {M_PI / 6, 0.2},
    {M_PI / 3, 1.0},
    {2.0 * M_PI / 3, 0.8},
    {0.1, 0.1},
    {M_PI - 0.1, 0.1},
  };

  for (const auto & seed : seeds) {
    double st_s, el_s;
    if (newton_solve(tx, tz, seed[0], seed[1], st_s, el_s)) {
      return st_out = st_s, el_out = el_s, true;
    }
  }
  return false;
}

  // -----------------------------------------------------------------------

geometry_msgs::msg::Pose Kinematics::forward_kinematics(
  const std::vector<double> & /*joint_angles_rad*/,
  const geometry_msgs::msg::TransformStamped & base_target_tf,
  const geometry_msgs::msg::TransformStamped & lift_target_tf,
  const bool is_right)
{
  geometry_msgs::msg::Pose move_pose;
    // Right arm reaches along base-frame y < 0, left along y > 0.
  const double side_sign = is_right ? -1.0 : 1.0;
    // Lateral offset of the EE reach line from base center (signed by arm side).
  const double ee_lateral = side_sign * (BaseToShoulderDY + EeLateralFromShoulder);

  const double tx_base = base_target_tf.transform.translation.x;
  const double ty_base = base_target_tf.transform.translation.y;

    // Yaw the base so the END-EFFECTOR reach line (not the shoulder pivot) is
    // aimed at the target. The arm extends in a plane offset by ee_lateral from
    // base center, so simply pointing base center (or the shoulder) at the target
    // leaves the EE short by that offset. Rotate base center toward the target,
    // then back off by asin(ee_lateral / range) so the offset line intersects it.
    //   after rotating the base by diff_yaw, the target's lateral coord in the
    //   new base frame equals ee_lateral (verified: residual ~0).
  const double range = std::hypot(tx_base, ty_base);
  double diff_yaw = 0.0;
  if (range > std::abs(ee_lateral)) {
    diff_yaw = std::atan2(ty_base, tx_base) -
      std::asin(std::clamp(ee_lateral / range, -1.0, 1.0));
  } else {
      // Target closer than the lateral offset: fall back to facing it directly.
    diff_yaw = std::atan2(ty_base, tx_base);
  }
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, diff_yaw);
  move_pose.orientation = tf2::toMsg(q);

  const double tx_lift = lift_target_tf.transform.translation.x;
  const double tz_lift = lift_target_tf.transform.translation.z;

    // Reposition deltas (dx forward base shift, dz lift raise) that bring an
    // unreachable target into the arm workspace. Driving the base forward by dx
    // and raising the lift by dz shifts the target to (tx - dx, tz - dz) in
    // body_lift_link, so we need ik_solve(tx_lift - dx, tz_lift - dz) to succeed.
    //
    // The reachable (x, z) set is a thin curved region, so dx and dz are coupled:
    // near the z extremes the reachable x-window collapses. Computing one axis
    // then the other (each assuming the other is in range) lands on the boundary
    // sliver and reports a correction that is not actually reachable. Instead we
    // search the (dz, dx) grid jointly and pick the reachable pair with the least
    // total motion, so the reported move_pose always corresponds to a target the
    // arm can reach after the move.
  double dx = 0.0;
  double dz = 0.0;
  {
    double s, e;
    if (!ik_solve(tx_lift, tz_lift, s, e)) {
        // Candidate dz values: 0 plus a sweep across the lift travel that maps
        // tz_lift into the z-workspace. dz>0 raises the lift (target moves down
        // in the lift frame), dz<0 lowers it.
      constexpr int NZ = 60;
      const double dz_lo = tz_lift - WS_Z_MAX;
      const double dz_hi = tz_lift - WS_Z_MIN;
        // Candidate dx values across the base travel that maps tx_lift into the
        // x-workspace, plus 0.
      constexpr int NX = 60;
      const double dx_lo = tx_lift - WS_X_MAX;
      const double dx_hi = tx_lift - WS_X_MIN;

        // 2 cm interior margin so the adjusted target lands safely inside the
        // workspace, not right on the (numerically fragile) boundary.
      constexpr double MARGIN = 0.02;

      double best_cost = std::numeric_limits<double>::infinity();
      double best_dx = 0.0, best_dz = 0.0;
      bool found = false;

        // Test one (dx, dz) pair: keep it if reachable and cheaper than the best
        // so far. Cost weights lift and base motion equally; both are costly.
      auto try_pair = [&](double cand_dx, double cand_dz) {
          const double ax = tx_lift - cand_dx;
          const double az = tz_lift - cand_dz;
          double cs, ce;
          if (!ik_solve(ax, az, cs, ce)) {return;}
          const double cost = std::abs(cand_dx) + std::abs(cand_dz);
          if (cost < best_cost) {
            best_cost = cost; best_dx = cand_dx; best_dz = cand_dz; found = true;
          }
        };

      for (int iz = 0; iz <= NZ; ++iz) {
        const double raw_dz = dz_lo + iz * (dz_hi - dz_lo) / NZ;
          // Apply the interior margin in the direction that pushes toward 0-side
          // interior of the band (sign follows raw_dz).
        const double cdz = raw_dz + (raw_dz >= 0.0 ? MARGIN : -MARGIN);
        for (int ix = 0; ix <= NX; ++ix) {
          const double raw_dx = dx_lo + ix * (dx_hi - dx_lo) / NX;
          const double cdx = raw_dx + (raw_dx >= 0.0 ? MARGIN : -MARGIN);
          try_pair(cdx, cdz);
        }
          // Also try this dz with no base shift (target already in x-range).
        try_pair(0.0, cdz);
      }
        // And try every dx with no lift change.
      for (int ix = 0; ix <= NX; ++ix) {
        const double raw_dx = dx_lo + ix * (dx_hi - dx_lo) / NX;
        const double cdx = raw_dx + (raw_dx >= 0.0 ? MARGIN : -MARGIN);
        try_pair(cdx, 0.0);
      }

      if (found) {
        dx = best_dx;
        dz = best_dz;
      }
    }
  }

  move_pose.position.x = dx;    // base forward shift (+ = forward, - = backward)
  move_pose.position.y = 0.0;
  move_pose.position.z = dz;    // lift delta (+ = up, - = down)

  return move_pose;
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
  const geometry_msgs::msg::TransformStamped & lift_target_tf,
  const bool /*is_right*/)
{
  const double tx = lift_target_tf.transform.translation.x;
  const double tz = lift_target_tf.transform.translation.z;

  double shoulder_tilt, elbow;
  if (!ik_solve(tx, tz, shoulder_tilt, elbow)) {
    return {};
  }

  const double upper_roll = -M_PI_2;
  const double wrist_tilt = M_PI_2 - shoulder_tilt - elbow;

  return {shoulder_tilt, upper_roll, 0.0, elbow, 0.0, wrist_tilt, 0.0};
}

  // -----------------------------------------------------------------------

std::vector<double> Kinematics::look_at(
  const geometry_msgs::msg::TransformStamped & target_tf)
{
  const double tx = target_tf.transform.translation.x;
  const double ty = target_tf.transform.translation.y;
  const double tz = target_tf.transform.translation.z;
  return {std::atan2(ty, tx), std::atan2(tz, std::hypot(tx, ty))};
}

} // namespace sobit_home
