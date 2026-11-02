#!/usr/bin/env python3
"""Straight-line / pure-rotation step test for the sobit_home swerve base drive loop.

Publishes a bounded cmd_vel step and logs odom, the two wheel controller command
topics, and per-wheel position/velocity/effort/temperature from dynamic_joint_states,
to analyse the RoboMaster M3508 drive loop and the Dynamixel steering gate.

THIS MOVES THE ROBOT (>80 kg, real hardware). It never launches anything and only
talks to topics on an already-running bringup.

SAFETY:
  1. Hard CLI caps: |vx| <= 0.3 m/s, |vy| <= 0.2 m/s, |wz| <= 0.5 rad/s, distance <= 3.0 m,
     angle <= 6.4 rad, timeout <= 45 s.
  2. Exactly one of --vx/--vy/--wz may be non-zero (no diagonal or combined motion); the
     other two Twist fields are always forced to exactly 0.0 before publishing.
  3. WAIT_DATA gate: won't publish a non-zero command until odom, dynamic_joint_states,
     and both wheel command echoes have all been seen (5 s timeout, names what's missing).
  4. RUN stops on: target distance/angle reached, --timeout elapsed, odom or
     dynamic_joint_states stale (> 0.3 s / > 0.5 s), any drive temp >= 80 degC,
     drift off the starting heading > 0.3 m (perpendicular for translate/rotate runs,
     longitudinal -- i.e. along the intended direction -- for lateral runs), or all four
     drive joints' feedback bit-identical for > 0.5 s while commanding motion (e-stop can
     freeze dynamic_joint_states without making it stale).
  5. rclpy is initialised with SignalHandlerOptions.NO: Ctrl-C and SIGTERM both raise a
     plain KeyboardInterrupt that unwinds through the same try/finally, so the ROS
     context stays valid long enough for the shutdown burst to actually publish.
  6. Every exit path runs a burst of 10 zero-Twist publishes over ~0.2 s before
     shutdown; if the context is already down it says so loudly instead of failing
     quietly -- use the e-stop.
  7. Without --yes, prints the plan and requires the operator to type "go".

Usage:
    source install/setup.bash
    ros2 run sobit_home_control base_step_test.py --vx 0.1 --distance 0.5
    ros2 run sobit_home_control base_step_test.py --vx -0.15 --distance 1.0 --ramp 0.5
    ros2 run sobit_home_control base_step_test.py --wz 0.3 --angle 1.0
    ros2 run sobit_home_control base_step_test.py --vy 0.1 --distance 0.3
    python3 base_step_test.py --ns sobit_home --vx 0.1 --distance 1.0 --yes --out ~/log.csv

CSV columns: one row per 50 Hz control tick (latest-value sampling), t (seconds, relative
to the start of RUN; negative during SETTLE), t_mono (time.monotonic() of the tick), state,
cmd_vx, cmd_vy, cmd_wz, odom x/y/yaw, odom qz/qw (raw orientation quaternion), odom twist
vx/vy/wz, displacement from the run-start pose, acc_yaw (accumulated yaw, rad, keeps
accumulating through POST), then per wheel (f_l, f_r, b_l, b_r): drive_cmd [rad/s],
drive_vel [rad/s], drive_effort [N*m], drive_temp [degC], steer_goal [rad], steer_pos
[rad], drive_pos [rad, cumulative], steer_vel [rad/s], steer_effort [N*m]; finally the
age of the last dynamic_joint_states message.
"""
import argparse
import csv
import math
import os
import signal
import sys
import time
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

# --- Fixed physical / kinematic facts (see swerve_config.yaml) --------------------------
WHEEL_RADIUS_M = 0.075
# Joint order matches controllers.yaml (steer & drive controllers both list
# f_l, f_r, b_l, b_r), so the command arrays are indexed the same way.
WHEELS = ("f_l", "f_r", "b_l", "b_r")
DRIVE_JOINT_NAMES = {w: f"wheel_drive_{w}_joint" for w in WHEELS}
STEER_JOINT_NAMES = {w: f"wheel_steer_{w}_joint" for w in WHEELS}

# --- Hard, non-overridable safety caps ---------------------------------------------------
HARD_MAX_VX = 0.3          # m/s
HARD_MAX_VY = 0.2          # m/s
HARD_MAX_WZ = 0.5          # rad/s
HARD_MAX_DISTANCE = 3.0    # m
HARD_MAX_ANGLE = 6.4       # rad, a full 360 deg turn plus margin
HARD_MAX_TIMEOUT = 45.0    # s

STALE_ODOM_S = 0.3         # s
STALE_DYN_JS_S = 0.5       # s
FROZEN_FEEDBACK_S = 0.5    # s, bit-identical drive feedback while commanding motion
MAX_DRIVE_TEMP_C = 80.0    # degC
MAX_LATERAL_DRIFT_M = 0.3  # m
WAIT_DATA_TIMEOUT_S = 5.0  # s

TICK_HZ = 50.0
TICK_PERIOD_S = 1.0 / TICK_HZ

# Diagnostic thresholds used only for the printed summary, not for stopping the robot.
FIRST_MOTION_VEL_THRESH = 0.1     # rad/s
FIRST_MOTION_ODOM_THRESH_M = 0.005  # m
PID_RESET_HIGH_EFFORT = 0.3       # N*m
PID_RESET_LOW_EFFORT = 0.05       # N*m
PID_RESET_MAX_TICK_GAP = 2


def normalize_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def dyn_lookup(msg, joint_name, interface_name):
    """Look up (joint, interface) in a DynamicJointState by name, not index. NaN if absent."""
    if msg is None:
        return math.nan
    try:
        j = list(msg.joint_names).index(joint_name)
    except ValueError:
        return math.nan
    iv = msg.interface_values[j]
    try:
        k = list(iv.interface_names).index(interface_name)
    except ValueError:
        return math.nan
    return iv.values[k]


def lstsq_slope(xs, ys):
    """Least-squares slope of ys vs xs (standard library only). None if underdetermined."""
    n = len(xs)
    if n < 2:
        return None
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    den = sum((x - mean_x) ** 2 for x in xs)
    if den <= 0.0:
        return None
    num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    return num / den


class _SigtermToKeyboardInterrupt:
    """Converts SIGTERM into a KeyboardInterrupt so it unwinds through the same
    try/finally as Ctrl-C, guaranteeing the zero-velocity shutdown burst still runs."""

    def __enter__(self):
        self._prev = signal.signal(signal.SIGTERM, self._handler)
        return self

    def _handler(self, signum, frame):
        raise KeyboardInterrupt("SIGTERM")

    def __exit__(self, exc_type, exc, tb):
        signal.signal(signal.SIGTERM, self._prev)
        return False


class BaseStepTest(Node):
    """State machine: WAIT_DATA -> SETTLE -> RUN -> POST -> DONE, ticking at 50 Hz."""

    def __init__(self, args, mode):
        super().__init__("base_step_test")
        self.args = args
        self.mode = mode  # 'translate', 'lateral', or 'rotate'
        ns = args.ns

        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        # swerve_controller_main.cpp subscribes to cmd_vel with depth=1 RELIABLE VOLATILE;
        # match it so this publisher is never silently incompatible with that subscription.
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_cmd_vel = self.create_publisher(Twist, f"/{ns}/cmd_vel", pub_qos)

        # BEST_EFFORT requests match both BEST_EFFORT and RELIABLE publishers, so these
        # subscriptions can't silently fail to match on QoS.
        self.sub_odom = self.create_subscription(
            Odometry, f"/{ns}/odom", self._on_odom, sub_qos)
        self.sub_dyn = self.create_subscription(
            DynamicJointState, f"/{ns}/dynamic_joint_states", self._on_dyn, sub_qos)
        self.sub_drive_cmd = self.create_subscription(
            Float64MultiArray, f"/{ns}/wheel_drive_velocity_controller/commands",
            self._on_drive_cmd, sub_qos)
        self.sub_steer_cmd = self.create_subscription(
            Float64MultiArray, f"/{ns}/wheel_steer_position_controller/commands",
            self._on_steer_cmd, sub_qos)

        self.last_odom = None
        self.last_odom_recv_mono = None
        self.last_dyn = None
        self.last_dyn_recv_mono = None
        self.last_drive_cmd = None   # list[4] rad/s, WHEELS order
        self.last_steer_cmd = None   # list[4] rad, WHEELS order

        # Drive feedback-freeze tracking (see _check_run_stop): tuple of
        # (position, velocity, effort) per drive joint, and when it last changed.
        self.last_feedback_tuple = None
        self.last_feedback_change_mono = None

        self.rows = []
        self.state = "WAIT_DATA"
        self.stop_reason = None

        now = time.monotonic()
        self.t_wait_start = now
        self.t_state_start = now
        self.t_run_start = None

        self.baseline_pose = None     # (x, y, yaw) captured at first SETTLE tick
        self.run_start_pose = None    # (x, y, yaw) captured at RUN entry
        self.prev_yaw = None
        self.accumulated_yaw = 0.0
        self.displacement = 0.0
        self.lateral_drift = 0.0
        self.longitudinal_drift = 0.0

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        self.timer = self.create_timer(TICK_PERIOD_S, self._tick)

    # --- subscription callbacks ----------------------------------------------------------
    def _on_odom(self, msg):
        self.last_odom = msg
        self.last_odom_recv_mono = time.monotonic()

    def _on_dyn(self, msg):
        self.last_dyn = msg
        self.last_dyn_recv_mono = time.monotonic()

    def _on_drive_cmd(self, msg):
        self.last_drive_cmd = list(msg.data)

    def _on_steer_cmd(self, msg):
        self.last_steer_cmd = list(msg.data)

    # --- helpers ---------------------------------------------------------------------------
    def _have_all_topics(self):
        return (self.last_odom is not None and self.last_dyn is not None
                and self.last_drive_cmd is not None and self.last_steer_cmd is not None)

    def _max_drive_temp(self):
        if self.last_dyn is None:
            return None
        temps = [dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "temperature") for w in WHEELS]
        temps = [t for t in temps if not math.isnan(t)]
        return max(temps) if temps else None

    def _publish_cmd(self, vx, vy, wz):
        # Force the two inactive axes to exactly 0.0, regardless of what the caller passed.
        if self.mode != "translate":
            vx = 0.0
        if self.mode != "lateral":
            vy = 0.0
        if self.mode != "rotate":
            wz = 0.0
        self.cmd_vx, self.cmd_vy, self.cmd_wz = vx, vy, wz
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz
        self.pub_cmd_vel.publish(msg)

    def _update_odom_derived(self):
        if self.last_odom is None:
            return
        x = self.last_odom.pose.pose.position.x
        y = self.last_odom.pose.pose.position.y
        yaw = quat_to_yaw(self.last_odom.pose.pose.orientation)

        if self.state == "SETTLE" and self.baseline_pose is None:
            self.baseline_pose = (x, y, yaw)

        ref = self.run_start_pose or self.baseline_pose
        if ref is not None:
            rx, ry, ryaw = ref
            dx, dy = x - rx, y - ry
            self.displacement = math.hypot(dx, dy)
            # component perpendicular to the heading recorded at RUN start
            self.lateral_drift = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
            # component along the heading recorded at RUN start
            self.longitudinal_drift = dx * math.cos(ryaw) + dy * math.sin(ryaw)

        if self.state in ("RUN", "POST"):
            # Keep accumulating through POST too: the robot coasts after the stop command.
            if self.prev_yaw is not None:
                self.accumulated_yaw += normalize_angle(yaw - self.prev_yaw)
            self.prev_yaw = yaw

    def _log_row(self, now_mono):
        if self.state == "SETTLE":
            t = now_mono - (self.t_state_start + self.args.settle)
        else:
            t = now_mono - self.t_run_start

        odom = self.last_odom
        row = {
            "t": t,
            "t_mono": now_mono,
            "state": self.state,
            "cmd_vx": self.cmd_vx,
            "cmd_vy": self.cmd_vy,
            "cmd_wz": self.cmd_wz,
            "odom_x": odom.pose.pose.position.x if odom else math.nan,
            "odom_y": odom.pose.pose.position.y if odom else math.nan,
            "odom_yaw": quat_to_yaw(odom.pose.pose.orientation) if odom else math.nan,
            "odom_qz": odom.pose.pose.orientation.z if odom else math.nan,
            "odom_qw": odom.pose.pose.orientation.w if odom else math.nan,
            "odom_vx": odom.twist.twist.linear.x if odom else math.nan,
            "odom_vy": odom.twist.twist.linear.y if odom else math.nan,
            "odom_wz": odom.twist.twist.angular.z if odom else math.nan,
            "displacement": self.displacement,
            "acc_yaw": self.accumulated_yaw,
        }
        for i, w in enumerate(WHEELS):
            row[f"{w}_drive_cmd"] = self.last_drive_cmd[i] if self.last_drive_cmd else math.nan
            row[f"{w}_drive_vel"] = dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "velocity")
            row[f"{w}_drive_effort"] = dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "effort")
            row[f"{w}_drive_temp"] = dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "temperature")
            row[f"{w}_steer_goal"] = self.last_steer_cmd[i] if self.last_steer_cmd else math.nan
            row[f"{w}_steer_pos"] = dyn_lookup(self.last_dyn, STEER_JOINT_NAMES[w], "position")
            row[f"{w}_drive_pos"] = dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "position")
            row[f"{w}_steer_vel"] = dyn_lookup(self.last_dyn, STEER_JOINT_NAMES[w], "velocity")
            row[f"{w}_steer_effort"] = dyn_lookup(self.last_dyn, STEER_JOINT_NAMES[w], "effort")
        row["dyn_js_age"] = (now_mono - self.last_dyn_recv_mono
                             if self.last_dyn_recv_mono is not None else math.nan)
        self.rows.append(row)

    def _ramped_cmd(self, elapsed_run):
        scale = 1.0 if self.args.ramp <= 0.0 else min(1.0, elapsed_run / self.args.ramp)
        return self.args.vx * scale, self.args.vy * scale, self.args.wz * scale

    # --- state transitions -----------------------------------------------------------------
    def _enter_settle(self, now_mono):
        self.state = "SETTLE"
        self.t_state_start = now_mono

    def _enter_run(self, now_mono):
        self.state = "RUN"
        self.t_state_start = now_mono
        self.t_run_start = now_mono
        if self.last_odom is not None:
            x = self.last_odom.pose.pose.position.x
            y = self.last_odom.pose.pose.position.y
            yaw = quat_to_yaw(self.last_odom.pose.pose.orientation)
            self.run_start_pose = (x, y, yaw)
            self.prev_yaw = yaw
        self.accumulated_yaw = 0.0
        self.last_feedback_tuple = None
        self.last_feedback_change_mono = None

    def _enter_post(self, now_mono):
        self.state = "POST"
        self.t_state_start = now_mono
        self._publish_cmd(0.0, 0.0, 0.0)

    def _finish(self):
        self._publish_cmd(0.0, 0.0, 0.0)
        self.state = "DONE"

    def _check_run_stop(self, now_mono):
        if now_mono - self.last_odom_recv_mono > STALE_ODOM_S:
            return "odom_stale"
        if now_mono - self.last_dyn_recv_mono > STALE_DYN_JS_S:
            return "joint_states_stale"
        # e-stop can freeze dynamic_joint_states at its last value without making it stale.
        # Only checked past RUN's first second, when the steering gate may hold drives at 0.
        cmd_nonzero = (self.cmd_vx != 0.0) or (self.cmd_vy != 0.0) or (self.cmd_wz != 0.0)
        if cmd_nonzero and now_mono - self.t_run_start > 1.0:
            feedback = tuple(
                (dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "position"),
                 dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "velocity"),
                 dyn_lookup(self.last_dyn, DRIVE_JOINT_NAMES[w], "effort"))
                for w in WHEELS)
            has_nan = any(math.isnan(v) for triple in feedback for v in triple)
            if has_nan or feedback != self.last_feedback_tuple:
                # NaN (missing data) is treated as always-changed so it can never look
                # "frozen"; the stale checks above are what catch actually-missing data.
                self.last_feedback_tuple = feedback
                self.last_feedback_change_mono = now_mono
            elif now_mono - self.last_feedback_change_mono > FROZEN_FEEDBACK_S:
                return "feedback_frozen"
        max_temp = self._max_drive_temp()
        if max_temp is not None and max_temp >= MAX_DRIVE_TEMP_C:
            return "over_temperature"
        if self.mode in ("translate", "lateral"):
            if self.displacement >= self.args.distance:
                return "distance_reached"
        else:
            if abs(self.accumulated_yaw) >= self.args.angle:
                return "angle_reached"
        if self.mode == "lateral":
            if abs(self.longitudinal_drift) > MAX_LATERAL_DRIFT_M:
                return "longitudinal_drift"
        else:
            if abs(self.lateral_drift) > MAX_LATERAL_DRIFT_M:
                return "lateral_drift"
        if now_mono - self.t_run_start >= self.args.timeout:
            return "timeout"
        return None

    # --- main 50 Hz tick ---------------------------------------------------------------------
    def _tick(self):
        now_mono = time.monotonic()

        if self.state == "WAIT_DATA":
            if self._have_all_topics():
                self._enter_settle(now_mono)
            elif now_mono - self.t_wait_start > WAIT_DATA_TIMEOUT_S:
                missing = []
                if self.last_odom is None:
                    missing.append(f"/{self.args.ns}/odom")
                if self.last_dyn is None:
                    missing.append(f"/{self.args.ns}/dynamic_joint_states")
                if self.last_drive_cmd is None:
                    missing.append(f"/{self.args.ns}/wheel_drive_velocity_controller/commands")
                if self.last_steer_cmd is None:
                    missing.append(f"/{self.args.ns}/wheel_steer_position_controller/commands")
                self.get_logger().error(
                    "Timed out after %.1fs waiting for: %s. Is bringup + the swerve "
                    "controller running?" % (WAIT_DATA_TIMEOUT_S, ", ".join(missing)))
                self.stop_reason = "wait_data_timeout"
                self._finish()
            return

        self._update_odom_derived()

        if self.state == "SETTLE":
            self._publish_cmd(0.0, 0.0, 0.0)
            self._log_row(now_mono)
            if now_mono - self.t_state_start >= self.args.settle:
                self._enter_run(now_mono)
            return

        if self.state == "RUN":
            elapsed_run = now_mono - self.t_run_start
            vx, vy, wz = self._ramped_cmd(elapsed_run)
            self._publish_cmd(vx, vy, wz)
            self._log_row(now_mono)
            reason = self._check_run_stop(now_mono)
            if reason:
                if reason == "feedback_frozen":
                    print("[SAFETY] drive feedback frozen (e-stop / motor power cut?) "
                          "-- stopping", file=sys.stderr)
                self.stop_reason = reason
                self._enter_post(now_mono)
            return

        if self.state == "POST":
            self._publish_cmd(0.0, 0.0, 0.0)
            self._log_row(now_mono)
            if now_mono - self.t_state_start >= self.args.settle:
                self._finish()
            return


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default="sobit_home", help="robot namespace (default: sobit_home)")
    ap.add_argument("--vx", type=float, default=None,
                     help="forward(+)/backward(-) speed [m/s], hard max |vx|=%.1f. Defaults "
                          "to 0.1 when --vx/--vy/--wz are all omitted, else defaults to 0."
                          % HARD_MAX_VX)
    ap.add_argument("--vy", type=float, default=0.0,
                     help="left(+)/right(-) lateral speed [m/s], hard max |vy|=%.1f. Exactly "
                          "one of --vx/--vy/--wz may be non-zero." % HARD_MAX_VY)
    ap.add_argument("--wz", type=float, default=0.0,
                     help="yaw rate [rad/s], hard max |wz|=%.1f. Non-zero --wz means a pure "
                          "in-place rotation: --vx and --vy must both be 0 in that case."
                          % HARD_MAX_WZ)
    ap.add_argument("--distance", type=float, default=1.0,
                     help="stop distance for a translate run [m], hard max %.1f"
                          % HARD_MAX_DISTANCE)
    ap.add_argument("--angle", type=float, default=1.57,
                     help="stop angle for a rotate run [rad], hard max %.1f" % HARD_MAX_ANGLE)
    ap.add_argument("--timeout", type=float, default=15.0,
                     help="hard RUN timeout [s], hard max %.1f" % HARD_MAX_TIMEOUT)
    ap.add_argument("--ramp", type=float, default=0.0,
                     help="linear ramp-up time from 0 to target speed [s] (default: 0, "
                          "i.e. step input)")
    ap.add_argument("--settle", type=float, default=1.0,
                     help="zero-velocity settle time before RUN and after stopping [s]")
    ap.add_argument("--out", default=None,
                     help="output CSV path (default: ~/base_step_logs/"
                          "step_<timestamp>_vx<vx>.csv)")
    ap.add_argument("--yes", action="store_true",
                     help="skip the interactive 'go' confirmation")
    args = ap.parse_args()
    if args.vx is None:
        args.vx = 0.1 if (args.vy == 0.0 and args.wz == 0.0) else 0.0
    return args


def validate_args(args):
    errors = []
    if abs(args.vx) > HARD_MAX_VX:
        errors.append(f"|--vx|={abs(args.vx)} exceeds hard max {HARD_MAX_VX} m/s")
    if abs(args.vy) > HARD_MAX_VY:
        errors.append(f"|--vy|={abs(args.vy)} exceeds hard max {HARD_MAX_VY} m/s")
    if abs(args.wz) > HARD_MAX_WZ:
        errors.append(f"|--wz|={abs(args.wz)} exceeds hard max {HARD_MAX_WZ} rad/s")
    nonzero_axes = [name for name, val in
                     (("--vx", args.vx), ("--vy", args.vy), ("--wz", args.wz)) if val != 0.0]
    if not nonzero_axes:
        errors.append("--vx, --vy, and --wz are all 0: nothing to do")
    elif len(nonzero_axes) > 1:
        errors.append(f"{' and '.join(nonzero_axes)} are non-zero: pick exactly one axis, "
                       "no diagonal or combined motion")
    if not (0.0 < args.distance <= HARD_MAX_DISTANCE):
        errors.append(f"--distance must be in (0, {HARD_MAX_DISTANCE}] m")
    if not (0.0 < args.angle <= HARD_MAX_ANGLE):
        errors.append(f"--angle must be in (0, {HARD_MAX_ANGLE}] rad")
    if not (0.0 < args.timeout <= HARD_MAX_TIMEOUT):
        errors.append(f"--timeout must be in (0, {HARD_MAX_TIMEOUT}] s")
    if args.ramp < 0.0:
        errors.append("--ramp must be >= 0")
    if args.settle < 0.0:
        errors.append("--settle must be >= 0")
    if errors:
        sys.exit("base_step_test.py: refusing to run:\n  - " + "\n  - ".join(errors))


def default_out_path(args):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"step_{ts}_vx{args.vx:.2f}"
    if args.vy != 0.0:
        fname += f"_vy{args.vy:.2f}"
    if args.wz != 0.0:
        fname += f"_wz{args.wz:.2f}"
    return os.path.expanduser(f"~/base_step_logs/{fname}.csv")


def print_plan(args, mode, out_path):
    print("=" * 78)
    print("BASE STEP TEST -- this will move a >80 kg real robot")
    print("=" * 78)
    print(f"  namespace    : {args.ns}")
    if mode == "translate":
        direction = "forward" if args.vx > 0 else "backward"
        print(f"  mode         : straight line, {direction}")
        print(f"  commanded vx : {args.vx:+.3f} m/s (ramp {args.ramp:.2f} s)")
        print(f"  distance cap : {args.distance:.3f} m  <- RUN stop trigger")
    elif mode == "lateral":
        direction = "to the LEFT (+y)" if args.vy > 0 else "to the RIGHT (-y)"
        print(f"  mode         : pure lateral, {direction}")
        print(f"  commanded vy : {args.vy:+.3f} m/s (ramp {args.ramp:.2f} s)")
        print(f"  distance cap : {args.distance:.3f} m  <- RUN stop trigger")
    else:
        direction = "CCW (+)" if args.wz > 0 else "CW (-)"
        print(f"  mode         : pure in-place rotation, {direction}")
        print(f"  commanded wz : {args.wz:+.3f} rad/s (ramp {args.ramp:.2f} s)")
        print(f"  angle cap    : {args.angle:.3f} rad  <- RUN stop trigger")
    print(f"  timeout      : {args.timeout:.1f} s  <- also stops RUN")
    print(f"  settle       : {args.settle:.2f} s before and after RUN")
    drift_desc = ("longitudinal drift" if mode == "lateral" else "lateral drift")
    print(f"  other stop conditions: odom stale > {STALE_ODOM_S}s, dynamic_joint_states "
          f"stale > {STALE_DYN_JS_S}s, any drive temp >= {MAX_DRIVE_TEMP_C} degC, "
          f"{drift_desc} > {MAX_LATERAL_DRIFT_M} m, drive feedback frozen > "
          f"{FROZEN_FEEDBACK_S}s")
    print(f"  output CSV   : {out_path}")
    print("=" * 78)


def confirm(args):
    if args.yes:
        return
    if not sys.stdin.isatty():
        sys.exit("base_step_test.py: not an interactive terminal; pass --yes to proceed.")
    resp = input("Type 'go' to start, anything else to abort: ").strip()
    if resp != "go":
        sys.exit("Aborted by operator.")


def publish_zero_burst(node):
    """Unconditional safety net: called on every exit path via try/finally."""
    if node is None:
        return
    if not rclpy.ok():
        print("[SAFETY] context already shut down -- ZERO TWIST NOT SENT, use the e-stop",
              file=sys.stderr)
        return
    zero = Twist()
    try:
        for _ in range(10):
            node.pub_cmd_vel.publish(zero)
            time.sleep(0.02)
    except Exception as exc:  # noqa: BLE001 - this is the last line of defense
        print(f"[SAFETY] zero-velocity burst raised {exc!r}", file=sys.stderr)


CSV_FIELDNAMES = (
    ["t", "t_mono", "state", "cmd_vx", "cmd_vy", "cmd_wz", "odom_x", "odom_y", "odom_yaw",
     "odom_qz", "odom_qw", "odom_vx", "odom_vy", "odom_wz", "displacement", "acc_yaw"]
    + [f"{w}_{field}" for w in WHEELS
       for field in ("drive_cmd", "drive_vel", "drive_effort", "drive_temp",
                     "steer_goal", "steer_pos", "drive_pos", "steer_vel", "steer_effort")]
    + ["dyn_js_age"]
)


def write_csv(out_path, rows):
    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print(f"\n[LOG] wrote {len(rows)} rows to {out_path}")


def _mean(xs):
    return sum(xs) / len(xs) if xs else math.nan


def print_summary(rows, args, stop_reason):
    run_rows = [r for r in rows if r["state"] == "RUN"]
    # RUN + POST so the coast after the stop command is reflected in the final numbers.
    coast_rows = [r for r in rows if r["state"] in ("RUN", "POST")]
    print("\n" + "=" * 78)
    print("SUMMARY (RUN phase only)")
    print("=" * 78)
    print(f"  stop reason        : {stop_reason}")
    final_disp = coast_rows[-1]["displacement"] if coast_rows else math.nan
    final_acc_yaw_deg = math.degrees(coast_rows[-1]["acc_yaw"]) if coast_rows else math.nan
    print(f"  final displacement : {final_disp:.4f} m")
    print(f"  final accumulated yaw : {final_acc_yaw_deg:.2f} deg")

    if not run_rows:
        print("  no RUN-phase samples were recorded; nothing further to analyse.")
        return

    if args.vx != 0.0:
        expected = args.vx / WHEEL_RADIUS_M
        print(f"  expected wheel speed (vx / {WHEEL_RADIUS_M} m): {expected:+.3f} rad/s")
    elif args.vy != 0.0:
        expected = abs(args.vy) / WHEEL_RADIUS_M
        print(f"  expected wheel speed (|vy| / {WHEEL_RADIUS_M} m): {expected:.3f} rad/s")
    else:
        print("  expected wheel speed: n/a (pure rotation; vx/R formula assumes translation)")

    n_ticks = len(run_rows)
    gate_ticks = 0
    longest_gate_streak = 0
    cur_streak = 0
    for r in run_rows:
        cmd_nonzero = (r["cmd_vx"] != 0.0) or (r["cmd_vy"] != 0.0) or (r["cmd_wz"] != 0.0)
        all_drive_zero = all(r[f"{w}_drive_cmd"] == 0.0 for w in WHEELS)
        if cmd_nonzero and all_drive_zero:
            gate_ticks += 1
            cur_streak += 1
            longest_gate_streak = max(longest_gate_streak, cur_streak)
        else:
            cur_streak = 0
    print(f"  steering-gate active: {100.0 * gate_ticks / n_ticks:.1f}% of RUN ticks, "
          f"longest streak {longest_gate_streak / TICK_HZ:.2f} s")

    odom_first_motion_t = None
    for r in run_rows:
        if r["displacement"] > FIRST_MOTION_ODOM_THRESH_M:
            odom_first_motion_t = r["t"]
            break
    print(f"  time-to-first-motion (odom > {1000*FIRST_MOTION_ODOM_THRESH_M:.0f} mm): "
          f"{'%.3f s' % odom_first_motion_t if odom_first_motion_t is not None else 'never'}")

    last_30pct_start_idx = int(math.floor(0.7 * n_ticks))
    tail_rows = run_rows[last_30pct_start_idx:] or run_rows[-1:]

    print(f"  {'wheel':6s} {'t_first_mot[s]':>14s} {'peak|eff|':>10s} {'mean|eff|':>10s} "
          f"{'stall_slope[Nm/s]':>18s} {'max|steer_err|':>15s} {'PID-reset_events':>16s} "
          f"{'vel_ss_err':>10s}")
    for w in WHEELS:
        cmd_key, vel_key, eff_key = f"{w}_drive_cmd", f"{w}_drive_vel", f"{w}_drive_effort"

        t_first_motion = None
        for r in run_rows:
            if abs(r[vel_key]) > FIRST_MOTION_VEL_THRESH:
                t_first_motion = r["t"]
                break

        efforts = [abs(r[eff_key]) for r in run_rows if not math.isnan(r[eff_key])]
        peak_eff = max(efforts) if efforts else math.nan
        mean_eff = _mean(efforts) if efforts else math.nan

        if t_first_motion is None:
            stall_rows = run_rows
        else:
            stall_rows = [r for r in run_rows if r["t"] < t_first_motion]
        stall_xs = [r["t"] for r in stall_rows if not math.isnan(r[eff_key])]
        stall_ys = [r[eff_key] for r in stall_rows if not math.isnan(r[eff_key])]
        stall_slope = lstsq_slope(stall_xs, stall_ys)

        steer_errs = [abs(r[f"{w}_steer_goal"] - r[f"{w}_steer_pos"]) for r in run_rows
                      if not (math.isnan(r[f"{w}_steer_goal"]) or math.isnan(r[f"{w}_steer_pos"]))]
        max_steer_err = max(steer_errs) if steer_errs else math.nan

        pid_reset_events = 0
        for i, r in enumerate(run_rows):
            if math.isnan(r[eff_key]) or abs(r[eff_key]) <= PID_RESET_HIGH_EFFORT:
                continue
            for gap in range(1, PID_RESET_MAX_TICK_GAP + 1):
                j = i + gap
                if j >= len(run_rows):
                    break
                span = run_rows[i:j + 1]
                if any(math.isnan(x[eff_key]) for x in span):
                    continue
                if not all(x[cmd_key] != 0.0 for x in span):
                    continue
                if abs(run_rows[j][eff_key]) < PID_RESET_LOW_EFFORT:
                    pid_reset_events += 1
                    break

        ss_errs = [r[vel_key] - r[cmd_key] for r in tail_rows
                   if not (math.isnan(r[vel_key]) or math.isnan(r[cmd_key]))]
        vel_ss_err = _mean(ss_errs) if ss_errs else math.nan

        print(f"  {w:6s} "
              f"{('%.3f' % t_first_motion) if t_first_motion is not None else 'never':>14s} "
              f"{peak_eff:>10.3f} {mean_eff:>10.3f} "
              f"{('%.4f' % stall_slope) if stall_slope is not None else 'n/a':>18s} "
              f"{max_steer_err:>15.4f} {pid_reset_events:>16d} {vel_ss_err:>10.3f}")

    max_temp = max((r[f"{w}_drive_temp"] for r in run_rows for w in WHEELS
                    if not math.isnan(r[f"{w}_drive_temp"])), default=math.nan)
    print(f"  max drive temperature (RUN phase): {max_temp:.1f} degC")
    print("=" * 78)


def main():
    args = parse_args()
    validate_args(args)
    mode = "rotate" if args.wz != 0.0 else ("lateral" if args.vy != 0.0 else "translate")
    out_path = os.path.expanduser(args.out) if args.out else default_out_path(args)

    print_plan(args, mode, out_path)
    confirm(args)

    # NO signal_handler_options: rclpy installs no SIGINT/SIGTERM handler of its own, so
    # Ctrl-C stays a plain KeyboardInterrupt and the context is still valid in `finally`.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        with _SigtermToKeyboardInterrupt():
            node = BaseStepTest(args, mode)
            try:
                while rclpy.ok() and node.state != "DONE":
                    rclpy.spin_once(node, timeout_sec=0.02)
            except KeyboardInterrupt:
                node.stop_reason = node.stop_reason or "keyboard_interrupt_or_sigterm"
            except ExternalShutdownException:
                node.stop_reason = node.stop_reason or "external_shutdown"
    finally:
        publish_zero_burst(node)
        rows = node.rows if node is not None else []
        stop_reason = node.stop_reason if node is not None else "init_failed"
        try:
            write_csv(out_path, rows)
        except Exception as exc:  # noqa: BLE001
            print(f"Failed to write CSV to {out_path}: {exc}", file=sys.stderr)
        try:
            print_summary(rows, args, stop_reason)
        except Exception as exc:  # noqa: BLE001
            print(f"Failed to compute summary: {exc}", file=sys.stderr)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
