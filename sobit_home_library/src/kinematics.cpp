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
  // Base shift: find the minimum forward base displacement (dx, metres) that
  // brings an unreachable target (tx, tz) into the arm workspace.
  //
  // Driving forward dx shifts the target to (tx - dx, tz) in the arm frame.
  // Returns true and sets dx_out on success; false if tz is out of range
  // for all possible x (lift height adjustment required instead).
  // -----------------------------------------------------------------------
  bool Kinematics::base_shift_for_reachability(double tx, double tz, double &dx_out)
  {
    double dummy_st, dummy_el;
    if (ik_solve(tx, tz, dummy_st, dummy_el)) {
      dx_out = 0.0;
      return true;
    }

    // Check if tz is feasible at any x in the workspace
    constexpr int N_SCAN = 50;
    bool z_ok = false;
    for (int k = 0; k <= N_SCAN; ++k) {
      const double xk = WS_X_MIN + k * (WS_X_MAX - WS_X_MIN) / N_SCAN;
      if (ik_solve(xk, tz, dummy_st, dummy_el)) { z_ok = true; break; }
    }
    if (!z_ok) return false;

    // Coarse scan over the dx range [tx-WS_X_MAX, tx-WS_X_MIN] to find
    // any reachable dx, then bisect toward the smallest |dx|.
    const double dx_lo = tx - WS_X_MAX;
    const double dx_hi = tx - WS_X_MIN;

    // Scan forward (from dx=0 outward) to find first reachable dx
    double found_dx = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i <= N_SCAN; ++i) {
      const double dx = dx_lo + i * (dx_hi - dx_lo) / N_SCAN;
      if (ik_solve(tx - dx, tz, dummy_st, dummy_el)) { found_dx = dx; break; }
    }
    if (std::isnan(found_dx)) return false;

    // Bisect between 0 and found_dx to get the minimum |dx|
    double best_dx = found_dx;
    if (found_dx > 0.0) {
      double lo = 0.0, hi = found_dx;
      for (int i = 0; i < 40; ++i) {
        const double mid = (lo + hi) / 2.0;
        if (ik_solve(tx - mid, tz, dummy_st, dummy_el)) hi = mid;
        else lo = mid;
      }
      best_dx = hi;
    } else if (found_dx < 0.0) {
      double lo = found_dx, hi = 0.0;
      for (int i = 0; i < 40; ++i) {
        const double mid = (lo + hi) / 2.0;
        if (ik_solve(tx - mid, tz, dummy_st, dummy_el)) lo = mid;
        else hi = mid;
      }
      best_dx = lo;
    }

    // Also scan from the other end to check if opposite direction is smaller
    double found_dx2 = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i <= N_SCAN; ++i) {
      const double dx = dx_hi - i * (dx_hi - dx_lo) / N_SCAN;
      if (ik_solve(tx - dx, tz, dummy_st, dummy_el)) { found_dx2 = dx; break; }
    }
    if (!std::isnan(found_dx2) && std::abs(found_dx2) < std::abs(best_dx)) {
      if (found_dx2 > 0.0) {
        double lo = 0.0, hi = found_dx2;
        for (int i = 0; i < 40; ++i) {
          const double mid = (lo + hi) / 2.0;
          if (ik_solve(tx - mid, tz, dummy_st, dummy_el)) hi = mid;
          else lo = mid;
        }
        if (std::abs(hi) < std::abs(best_dx)) best_dx = hi;
      } else if (found_dx2 < 0.0) {
        double lo = found_dx2, hi = 0.0;
        for (int i = 0; i < 40; ++i) {
          const double mid = (lo + hi) / 2.0;
          if (ik_solve(tx - mid, tz, dummy_st, dummy_el)) lo = mid;
          else hi = mid;
        }
        if (std::abs(lo) < std::abs(best_dx)) best_dx = lo;
      }
    }

    dx_out = best_dx;
    return true;
  }

  // -----------------------------------------------------------------------

  geometry_msgs::msg::Pose Kinematics::forward_kinematics(
      const std::vector<double> & /*joint_angles_rad*/,
      const geometry_msgs::msg::TransformStamped &base_target_tf,
      const geometry_msgs::msg::TransformStamped &lift_target_tf,
      const bool is_right)
  {
    geometry_msgs::msg::Pose move_pose;
    const double side_sign  = is_right ? 1.0 : -1.0;
    const double shoulder_y = side_sign * BaseToShoulderDY;

    const double tx_base = base_target_tf.transform.translation.x;
    const double ty_base = base_target_tf.transform.translation.y;

    // Yaw the base to face the target
    const double diff_yaw = std::atan2(ty_base - shoulder_y, tx_base);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, diff_yaw);
    move_pose.orientation = tf2::toMsg(q);

    const double tx_lift = lift_target_tf.transform.translation.x;
    const double tz_lift = lift_target_tf.transform.translation.z;

    // Lift delta: find minimum dz such that ik_solve(tx_lift, tz_lift - dz) succeeds.
    // Raising the lift by dz shifts the target to (tx, tz - dz) in body_lift_link.
    double dz = 0.0;
    {
      double dummy_st, dummy_el;
      if (!ik_solve(tx_lift, tz_lift, dummy_st, dummy_el)) {
        // Check if any z at this x is reachable (otherwise x is out of range)
        bool z_feasible = false;
        for (int k = 0; k <= 40; ++k) {
          const double zk = WS_Z_MIN + k * (WS_Z_MAX - WS_Z_MIN) / 40.0;
          if (ik_solve(tx_lift, zk, dummy_st, dummy_el)) { z_feasible = true; break; }
        }
        if (z_feasible) {
          // Find minimum |dz| such that ik_solve(tx, tz - dz) succeeds.
          // dz range spans [tz_lift - WS_Z_MAX, tz_lift - WS_Z_MIN].
          // The reachable island may not touch dz=0, so scan from near-zero outward.
          const double dz_lo = tz_lift - WS_Z_MAX;
          const double dz_hi = tz_lift - WS_Z_MIN;

          auto try_dz = [&](double d) {
            double s, e; return ik_solve(tx_lift, tz_lift - d, s, e);
          };

          // Scan from the end closer to 0 to find first reachable dz
          // (to get the minimum-magnitude solution)
          double best_dz = std::numeric_limits<double>::quiet_NaN();

          // Scan the full dz range to find any reachable point,
          // then bisect toward minimum |dz| from each side.
          constexpr int N = 120;
          // Scan from near-zero outward in both directions to find the first
          // reachable dz, which is the minimum-magnitude solution.
          auto scan_from_zero = [&](double range_start, double range_end) -> double {
            // range_start is close to 0, range_end is farther
            for (int i = 0; i <= N; ++i) {
              double d = range_start + i * (range_end - range_start) / N;
              if (try_dz(d)) {
                // Bisect between previous step (unreachable) and d (reachable)
                double prev = range_start + (i > 0 ? (i - 1) : 0) * (range_end - range_start) / N;
                double lo = prev, hi = d;
                if (lo > hi) std::swap(lo, hi);
                for (int j = 0; j < 40; ++j) {
                  double mid = (lo + hi) / 2.0;
                  if (try_dz(mid)) hi = mid; else lo = mid;
                }
                return hi;
              }
            }
            return std::numeric_limits<double>::quiet_NaN();
          };

          // Positive side: scan from 0 toward dz_hi
          if (dz_hi > 0.0) {
            double candidate = scan_from_zero(std::max(dz_lo, 0.0), dz_hi);
            if (!std::isnan(candidate) && (std::isnan(best_dz) || candidate < std::abs(best_dz)))
              best_dz = candidate;
          }

          // Negative side: scan from 0 toward dz_lo
          if (dz_lo < 0.0) {
            double candidate = scan_from_zero(std::min(dz_hi, 0.0), dz_lo);
            if (!std::isnan(candidate) && (std::isnan(best_dz) || std::abs(candidate) < std::abs(best_dz)))
              best_dz = candidate;
          }

          if (!std::isnan(best_dz)) {
            // Add a 2 cm interior margin so the adjusted target lands
            // safely inside the workspace boundary, not right on the edge.
            constexpr double MARGIN = 0.02;
            dz = best_dz + (best_dz >= 0.0 ? MARGIN : -MARGIN);
          }
        }
      }
    }
    move_pose.position.y = 0.0;
    move_pose.position.z = dz;  // lift delta (+ = up, - = down)

    // Base x-shift needed to bring (tx, adjusted_tz) into IK workspace
    double dx = 0.0;
    base_shift_for_reachability(tx_lift, tz_lift - dz, dx);
    move_pose.position.x = dx;

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
