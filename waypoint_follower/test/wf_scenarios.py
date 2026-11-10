#!/usr/bin/env python3
"""Drives waypoint_follower with synthetic inputs and checks what it commands.

Start the node, then run this against it:

    ros2 launch waypoint_follower waypoint_follower.launch.py
    python3 waypoint_follower/test/wf_scenarios.py

The node is fed /odom, /final_waypoint, /gps_state, /pva/ins_status and
/erp42_racing/feedback, and its output is read back off
/erp42_racing/control_command.
"""
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32, Float64
from rcl_interfaces.srv import GetParameters
from waypoint_follower_msgs.msg import Lane, Waypoint, State
from waypoint_follower_msgs.msg import ControlCommand, Feedback
from waypoint_follower_msgs.msg import LidarTopicMsg

INS_GOOD = 3


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def straight_path(n=120, ds=0.5, y=0.0):
    """Waypoints along +X with zero curvature."""
    wps = []
    for i in range(n):
        w = Waypoint()
        w.waypoint_index = i
        w.pose.pose.position.x = i * ds
        w.pose.pose.position.y = y
        w.ryaw = 0.0
        w.rk = 0.0
        w.mission_state = 0
        wps.append(w)
    return wps


def straight_then_curve(n_straight=40, n_curve=80, ds=0.5, radius=6.0):
    """Straight run-in followed by a constant-radius left turn.

    Gives the curvature profile something to brake for: the corner limit is
    sqrt(a_lat/k) = sqrt(2.5/0.167) = 3.9 m/s = 14 km/h.
    """
    wps = []
    x, y, psi = 0.0, 0.0, 0.0
    k = 1.0 / radius
    for i in range(n_straight + n_curve):
        w = Waypoint()
        w.waypoint_index = i
        w.pose.pose.position.x = x
        w.pose.pose.position.y = y
        w.ryaw = psi
        w.rk = 0.0 if i < n_straight else k
        w.mission_state = 0
        wps.append(w)
        curv = 0.0 if i < n_straight else k
        x += ds * math.cos(psi)
        y += ds * math.sin(psi)
        psi += curv * ds
    return wps


def straight_arc_straight(n_in=30, arc_deg=90.0, radius=3.0, n_out=80, ds=0.5):
    """Straight, a constant-radius left arc, then straight again.

    Returns the waypoints and the path distance at which the arc ends.
    """
    k = 1.0 / radius
    s_arc0 = n_in * ds
    s_arc1 = s_arc0 + math.radians(arc_deg) * radius
    total = s_arc1 + n_out * ds
    wps = []
    x, y, psi, s = 0.0, 0.0, 0.0, 0.0
    while s <= total:
        curv = k if s_arc0 <= s < s_arc1 else 0.0
        w = Waypoint()
        w.waypoint_index = len(wps)
        w.pose.pose.position.x = x
        w.pose.pose.position.y = y
        w.ryaw = psi
        w.rk = curv
        w.mission_state = 0
        wps.append(w)
        x += ds * math.cos(psi)
        y += ds * math.sin(psi)
        psi += curv * ds
        s += ds
    return wps, s_arc1


def pose_at(wps, s, ds=0.5):
    """Pose at path distance s, interpolated between waypoints."""
    i = min(int(s / ds), len(wps) - 2)
    f = s / ds - i
    a, b = wps[i], wps[i + 1]
    x = a.pose.pose.position.x + f * (b.pose.pose.position.x - a.pose.pose.position.x)
    y = a.pose.pose.position.y + f * (b.pose.pose.position.y - a.pose.pose.position.y)
    return x, y, a.ryaw + f * (b.ryaw - a.ryaw)


class Harness(Node):
    def __init__(self):
        super().__init__('wf_tester')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
        self.lane_pub = self.create_publisher(Lane, '/final_waypoint', 1)
        self.state_pub = self.create_publisher(State, '/gps_state', 1)
        self.ins_pub = self.create_publisher(UInt32, '/pva/ins_status', 1)
        self.fb_pub = self.create_publisher(Feedback, '/erp42_racing/feedback', 1)
        self.lidar_pub = self.create_publisher(LidarTopicMsg, '/lidar_topic', 1)

        self.cmd = None
        self.create_subscription(
            ControlCommand, '/erp42_racing/control_command', self._cmd_cb, 1)
        self.ld = None
        self.create_subscription(
            Float64, '/wf_debug/lookahead', lambda m: setattr(self, 'ld', m.data), 1)

    def _cmd_cb(self, msg):
        self.cmd = msg

    def feed(self, wps, x, y, yaw, speed_mps, steer_rad=0.0, lane=0,
             duration=1.2, mission=0, ins_status=INS_GOOD, odom=True,
             wheels_follow_cmd=False, path=True, acc=None):
        """Publishes one set of inputs every 40 ms for `duration` seconds.

        wheels_follow_cmd reports the last command back as the measured wheel
        angle, the way serial_bridge would (feedback = -ControlCommand.steering),
        which closes the loop through the controller's feedback terms.
        acc=(acc_flag, acc_index_gap) also publishes /lidar_topic; the node
        keeps the last one it got, so send (False, 0) afterwards to clear it.
        """
        end = time.time() + duration
        while time.time() < end:
            now = self.get_clock().now().to_msg()

            if acc is not None:
                lt = LidarTopicMsg()
                lt.acc_flag = bool(acc[0])
                lt.acc_index_gap = int(acc[1])
                self.lidar_pub.publish(lt)

            if path:
                lane_msg = Lane()
                lane_msg.header.stamp = now
                lane_msg.header.frame_id = 'map'
                lane_msg.waypoints = wps
                self.lane_pub.publish(lane_msg)

            st = State()
            st.current_state = mission
            st.lane_number = lane
            self.state_pub.publish(st)

            ins = UInt32()
            ins.data = ins_status
            self.ins_pub.publish(ins)

            fb = Feedback()
            fb.header.stamp = now
            fb.speed = float(speed_mps)
            if wheels_follow_cmd:
                fb.steering = -self.cmd.steering if self.cmd else 0.0
            else:
                fb.steering = float(steer_rad)
            self.fb_pub.publish(fb)

            if not odom:
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.04)
                continue

            od = Odometry()
            od.header.stamp = now
            od.header.frame_id = 'map'
            od.pose.pose.position.x = float(x)
            od.pose.pose.position.y = float(y)
            qx, qy, qz, qw = yaw_to_quat(yaw)
            od.pose.pose.orientation.x = qx
            od.pose.pose.orientation.y = qy
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw
            od.twist.twist.linear.x = float(speed_mps)
            self.odom_pub.publish(od)

            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.04)
        rclpy.spin_once(self, timeout_sec=0.1)
        return self.cmd


def course_csv(name='0.csv'):
    """A recorded course file: $WF_COURSE_DIR, else the waypoint_system package
    (installed share, then source tree). The checks that need it are skipped
    when none of these has the file."""
    env_dir = os.environ.get('WF_COURSE_DIR')
    if env_dir:
        return os.path.join(env_dir, name)
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('waypoint_system'), 'data', 'default', name)
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'waypoint_system', 'data', 'default', name)


def course_window(rows, start, count=200, state=None):
    """Waypoints as local_path_publisher sends them: `count` points from `start`,
    with the recorded rk and mission_state columns left as they are (or every
    mission_state set to `state`)."""
    wps = []
    for i in range(start, min(start + count, len(rows))):
        _, x, y, st, ryaw, rk = rows[i]
        w = Waypoint()
        w.waypoint_index = i
        w.pose.pose.position.x = x
        w.pose.pose.position.y = y
        w.mission_state = int(st) if state is None else state
        w.ryaw = ryaw
        w.rk = rk
        wps.append(w)
    return wps


def node_params(node, names, target='/waypoint_follower'):
    """Values of the follower's parameters (None where unavailable)."""
    cli = node.create_client(GetParameters, f'{target}/get_parameters')
    if not cli.wait_for_service(timeout_sec=3.0):
        return [None] * len(names)
    fut = cli.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    if fut.result() is None:
        return [None] * len(names)
    out = []
    for v in fut.result().values:
        out.append({1: v.bool_value, 2: v.integer_value, 3: v.double_value,
                    8: list(v.double_array_value)}.get(v.type))
    return out


def check(name, ok, detail):
    print(f'[{"OK  " if ok else "FAIL"}] {name:<46} {detail}')
    return ok


def main():
    rclpy.init()
    h = Harness()
    time.sleep(1.5)
    res = []

    sp = straight_path()

    # Sign convention: offset left of the path and offset right of it must give
    # opposite steering, and each must point back towards the path.
    c_left = h.feed(sp, x=10.0, y=+0.6, yaw=0.0, speed_mps=3.0)
    s_left = c_left.steering
    c_right = h.feed(sp, x=10.0, y=-0.6, yaw=0.0, speed_mps=3.0)
    s_right = c_right.steering
    res.append(check('offset L/R give opposite steering',
                     s_left * s_right < 0,
                     f'left={s_left:+.4f} right={s_right:+.4f} rad'))

    # Standstill with a large offset. The old law divided by raw speed, so this
    # is where it saturated at +-90 deg; the softening term must keep it sane.
    c_stop = h.feed(sp, x=10.0, y=+1.0, yaw=0.0, speed_mps=0.0)
    res.append(check('v=0 with 1.0m offset stays bounded',
                     abs(c_stop.steering) <= math.radians(20.5),
                     f'steer={math.degrees(c_stop.steering):+.2f} deg'))

    # Heading error alone, sitting on the line but pointing off it.
    c_yaw = h.feed(sp, x=10.0, y=0.0, yaw=math.radians(12.0), speed_mps=3.0)
    c_yaw2 = h.feed(sp, x=10.0, y=0.0, yaw=math.radians(-12.0), speed_mps=3.0)
    res.append(check('heading error L/R give opposite steering',
                     c_yaw.steering * c_yaw2.steering < 0,
                     f'+12deg={c_yaw.steering:+.4f} -12deg={c_yaw2.steering:+.4f}'))

    # Curvature profile over a 3 m radius bend starting at x=20. Its own limit is
    # sqrt(a_lat/k) = sqrt(3.0/0.333) = 3.0 m/s = 10.8 km/h, and at a_dec = 1.5
    # the backward pass should start biting a few metres before the corner.
    curve = straight_then_curve(radius=3.0)

    c_far = h.feed(curve, x=2.0, y=0.0, yaw=0.0, speed_mps=4.0, duration=2.5)
    v_far = c_far.speed * 3.6
    res.append(check('no premature braking 18m before bend',
                     v_far > 14.0,
                     f'{v_far:.1f} km/h (mission >= 15)'))

    c_appr = h.feed(curve, x=18.5, y=0.0, yaw=0.0, speed_mps=4.0, duration=1.5)
    v_appr = c_appr.speed * 3.6
    res.append(check('braking begins BEFORE entering the bend',
                     v_appr < v_far - 0.5,
                     f'1.5m out={v_appr:.1f} km/h vs {v_far:.1f} km/h'))

    # Sit exactly on a waypoint well inside the arc so the curvature limit is the
    # only thing acting; any lateral offset would stack the CTE cut on top.
    wp_in = curve[70]
    c_in = h.feed(curve, x=wp_in.pose.pose.position.x, y=wp_in.pose.pose.position.y,
                  yaw=wp_in.ryaw, speed_mps=3.0, duration=1.5)
    v_in = c_in.speed * 3.6
    res.append(check('inside the bend held at the lateral-accel limit',
                     9.8 < v_in < 11.8,
                     f'{v_in:.1f} km/h (sqrt(3.0/0.333)=10.8 expected)'))

    # Cross-track slowdown, no lane change in progress.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=4.0, duration=2.5)
    c_on = h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=4.0, duration=1.5)
    v_on = c_on.speed * 3.6
    c_off = h.feed(sp, x=10.0, y=1.5, yaw=0.0, speed_mps=4.0, duration=1.5)
    v_off = c_off.speed * 3.6
    res.append(check('large CTE cuts speed',
                     v_off < v_on * 0.95,
                     f'on-line={v_on:.1f} km/h  1.5m off={v_off:.1f} km/h'))

    # Same error, but arriving right after a lane change. The allowance should
    # keep the speed up instead of crawling through every swap.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=4.0, lane=0, duration=2.0)
    c_lc = h.feed(sp, x=10.0, y=1.5, yaw=0.0, speed_mps=4.0, lane=1, duration=1.5)
    v_lc = c_lc.speed * 3.6
    res.append(check('lane change keeps speed at same CTE',
                     v_lc > v_off * 1.05,
                     f'no-change={v_off:.1f} km/h  lane-change={v_lc:.1f} km/h'))

    # Oscillation. The heading is swung either side of the path while the car
    # stays ON the line, so the cross-track term contributes nothing and any
    # speed cut can only have come from the hunting detector. A lane change is a
    # large but one-sided error and produces no sign flips, which is exactly why
    # the detector counts flips rather than magnitude.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=4.0, duration=3.0)
    v_calm = h.cmd.speed * 3.6

    # The detector reads the measured wheel angle, so the wheels have to follow.
    end = time.time() + 3.0
    side = 1.0
    while time.time() < end:
        h.feed(sp, x=10.0, y=0.0, yaw=math.radians(20.0) * side,
               speed_mps=4.0, duration=0.16, wheels_follow_cmd=True)
        side = -side
    v_osc = h.cmd.speed * 3.6
    # The rear axle stays on the line but the front swings ~0.41 m out at 20 deg,
    # so a little of the cut is the cross-track term; the hunting detector at its
    # 0.6 floor is what takes 15 km/h down to roughly 8.
    res.append(check('steering hunting cuts speed',
                     v_osc < v_calm * 0.65,
                     f'calm={v_calm:.1f} km/h  hunting={v_osc:.1f} km/h'))

    # Closed steering loop. With the wheels reporting the angle actually
    # commanded, the lag and yaw-rate terms must damp the command or leave it
    # alone; an inverted feedback sign shows up here as amplification.
    yaw4 = math.radians(4.0)
    c_open = h.feed(sp, x=10.0, y=0.0, yaw=yaw4, speed_mps=3.0, duration=1.6)
    s_open = c_open.steering
    c_closed = h.feed(sp, x=10.0, y=0.0, yaw=yaw4, speed_mps=3.0, duration=2.4,
                      wheels_follow_cmd=True)
    s_closed = c_closed.steering
    ratio = s_closed / s_open if abs(s_open) > 1e-6 else float('nan')
    res.append(check('closed steering loop does not amplify',
                     s_open * s_closed > 0 and ratio <= 1.05,
                     f'open={math.degrees(s_open):+.2f} closed={math.degrees(s_closed):+.2f} '
                     f'deg ratio={ratio:.2f}'))

    # INS cap followed by recovery. The ramp runs after the cap, so the speed
    # climbs from 7 km/h instead of stepping straight back to the mission speed.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=2.0, duration=2.5, ins_status=0)
    v_capped = h.cmd.speed * 3.6
    c_rec = h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=2.0, duration=0.5)
    v_rec = c_rec.speed * 3.6
    # 0.5 s at a_acc 1.5 m/s^2 is 7 + 2.7 = 9.7 km/h; a step would give 15.
    res.append(check('INS recovery ramps up from the cap',
                     v_capped <= 7.01 and v_rec < 11.0,
                     f'capped={v_capped:.1f} km/h  0.5s after={v_rec:.1f} km/h'))

    # Lane-change allowance decays to nothing over hold_sec (4 s), so an error
    # that is still there afterwards gets the normal cross-track cut.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=4.0, lane=0, duration=2.0)
    h.feed(sp, x=10.0, y=1.5, yaw=0.0, speed_mps=4.0, lane=1, duration=4.5)
    v_late = h.cmd.speed * 3.6
    res.append(check('lane-change allowance expires',
                     abs(v_late - v_off) < 0.6,
                     f'4.5s after swap={v_late:.1f} km/h  no-change={v_off:.1f} km/h'))

    # ── Real course geometry (waypoint_system/data/default/0.csv) ────────────
    # The recorded rk column is noisy point to point (R ~2 m readings on
    # straights), so these two catch the curvature estimate regressing to it.
    course_path = course_csv('0.csv')
    rows = np.loadtxt(course_path, delimiter=',') if os.path.exists(course_path) else None
    if rows is None:
        print(f'[SKIP] {"real-course checks (4)":<46} {course_path} not found; set WF_COURSE_DIR')

    if rows is not None:
        # Straight in mission state 6 (15 km/h): R > 40 m for the next 25 m, but the
        # raw rk there reads as a bend that would cut the command to ~7 km/h. The
        # command must stay at the mission speed.
        i = 2010
        c_str = h.feed(course_window(rows, i - 5), x=rows[i][1], y=rows[i][2],
                       yaw=rows[i][4], speed_mps=15 / 3.6, duration=4.0)
        v_str = c_str.speed * 3.6
        res.append(check('real straight: no slowdown from rk noise',
                         v_str > 14.5, f'{v_str:.1f} km/h (mission >= 15)'))

        # Tightest corner of the course: apex at idx 707, R 4.56 m, limit
        # sqrt(3.0 * 4.56) = 13.3 km/h. The car sits 2 m before it at 3.7 m/s, which
        # is exactly the lag-compensation distance (0.55 s * 3.7 m/s), and is told
        # the mission speed is 15 so the curvature limit is the only thing acting.
        i = 687
        # The waypoints there are mission state 1 (8 km/h); anticipate_mission would
        # cap them, so they are relabelled to isolate the curvature limit.
        c_apex = h.feed(course_window(rows, i - 5, state=0), x=rows[i][1], y=rows[i][2],
                        yaw=rows[i][4], speed_mps=3.7, duration=4.0)
        v_apex = c_apex.speed * 3.6
        res.append(check('real tightest corner slowed to its limit',
                         12.0 < v_apex < 14.5, f'{v_apex:.1f} km/h (13.3 expected)'))

    # /odom goes silent while path, INS and wheel feedback keep arriving. The
    # loop is still running, so without the watchdog it keeps driving on a
    # frozen pose.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=2.0)
    v_alive = h.cmd.speed * 3.6
    c_dead = h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=1.5, odom=False)
    res.append(check('odom loss commands a stop',
                     v_alive > 1.0 and c_dead.speed == 0.0 and c_dead.brake >= 90,
                     f'alive={v_alive:.1f} km/h  dead={c_dead.speed * 3.6:.1f} km/h '
                     f'brake={c_dead.brake}'))

    # /final_waypoint goes silent while /odom keeps arriving (local_path_publisher
    # sends nothing when it finds no closest waypoint). Without the path
    # watchdog the car kept following the last path at mission speed.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=2.0)
    v_alive = h.cmd.speed * 3.6
    c_nopath = h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=1.5, path=False)
    res.append(check('path loss commands a stop, wheel centred',
                     v_alive > 1.0 and c_nopath.speed == 0.0 and c_nopath.brake >= 90
                     and c_nopath.steering == 0.0,
                     f'alive={v_alive:.1f} km/h  no path={c_nopath.speed * 3.6:.1f} km/h '
                     f'brake={c_nopath.brake} steer={c_nopath.steering:+.4f}'))

    # An empty path used to leave the wheel centred at mission speed: steering
    # returned 0 and the cross-track error kept its last value.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=2.0)
    c_empty = h.feed([], x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=1.0)
    res.append(check('empty path commands a stop',
                     c_empty.speed == 0.0 and c_empty.brake >= 90,
                     f'speed={c_empty.speed * 3.6:.1f} km/h brake={c_empty.brake}'))

    # ── Stanley path geometry (stanley.tangent_half_window_m) ────────────────
    # The recorded ryaw jitters ~1.7 deg point to point on 0.csv, and the
    # heading term passes it straight into the steering. Walk the car along a
    # real straight at 5 m/s (Stanley share 0.7), aligned with the +-1 m chord,
    # and look at the steering left over after a 7-sample moving average.
    # Measured: 0.45 deg with the chord tangent, 1.68 deg with ryaw (rollback).
    if rows is not None:
        def chord_yaw(i, w=10):
            a, c = rows[max(i - w, 0)], rows[min(i + w, len(rows) - 1)]
            return math.atan2(c[2] - a[2], c[1] - a[1])

        walk = []
        for i in range(2000, 2100, 2):
            c = h.feed(course_window(rows, i - 5), x=rows[i][1], y=rows[i][2],
                       yaw=chord_yaw(i), speed_mps=5.0, duration=0.25)
            walk.append(math.degrees(c.steering))
        hf = [walk[k] - np.mean(walk[max(0, k - 3):k + 4]) for k in range(len(walk))]
        res.append(check('real straight: no steering jitter from ryaw',
                         np.std(hf) < 0.9,
                         f'high-freq steer std={np.std(hf):.2f} deg (ryaw gives ~1.7)'))

    # Repeated waypoints have no direction; they must be skipped, not turned
    # into a zero-length segment that breaks the projection.
    dup = []
    for w in straight_path():
        dup += [w, w]
    s_dup_l = h.feed(dup, x=10.0, y=+0.6, yaw=0.0, speed_mps=5.0).steering
    s_dup_r = h.feed(dup, x=10.0, y=-0.6, yaw=0.0, speed_mps=5.0).steering
    s_plain = h.feed(sp, x=10.0, y=+0.6, yaw=0.0, speed_mps=5.0).steering
    res.append(check('repeated waypoints steer like the plain path',
                     s_dup_l * s_dup_r < 0 and abs(s_dup_l - s_plain) < 1e-3,
                     f'dup L={s_dup_l:+.4f} R={s_dup_r:+.4f} plain L={s_plain:+.4f} rad'))

    # A NaN anywhere in the path or pose used to reach the vehicle: the +-20 deg
    # clamp in PlatformConnector compares false against NaN. With the rollback
    # (tangent_half_window_m 0) a NaN waypoint far ahead is ignored instead.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=1.5)
    bad = straight_path()
    bad[60].pose.pose.position.y = float('nan')
    c_nan = h.feed(bad, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=1.0)
    res.append(check('NaN waypoint commands a stop',
                     c_nan.speed == 0.0 and c_nan.brake >= 90 and math.isfinite(c_nan.steering),
                     f'speed={c_nan.speed * 3.6:.1f} km/h brake={c_nan.brake} '
                     f'steer={c_nan.steering:+.4f}'))

    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=1.5)
    c_nanp = h.feed(sp, x=float('nan'), y=0.0, yaw=0.0, speed_mps=3.0, duration=1.0)
    res.append(check('NaN pose commands a stop, wheel centred',
                     c_nanp.speed == 0.0 and c_nanp.brake >= 90 and c_nanp.steering == 0.0,
                     f'speed={c_nanp.speed * 3.6:.1f} km/h brake={c_nanp.brake} '
                     f'steer={c_nanp.steering:+.4f}'))

    # ── Corner exit hold (speed.exit_hold_m) ─────────────────────────────────
    # Walk the car at 3 m/s through a 3 m radius arc (limit 10.8 km/h) and out
    # onto a straight. Without the hold the command starts climbing the moment
    # the rear axle leaves the arc (~+1.8 km/h per 0.3 m); with it the arc
    # limit stays in force for 5 m. Also runs right after the NaN pose above, so
    # it doubles as a check that the hold recovers from one.
    arc, s_exit = straight_arc_straight()
    after = []
    s = s_exit - 8.0
    while s < s_exit + 12.0:
        x, y, yaw = pose_at(arc, s)
        c = h.feed(arc, x=x, y=y, yaw=yaw, speed_mps=3.0, duration=0.1)
        after.append((s - s_exit, c.speed * 3.6))
        s += 0.3
    held = max(v for d, v in after if 0.5 <= d <= 2.5)
    released = min(v for d, v in after if d >= 10.0)
    res.append(check('corner limit held just past the exit',
                     held <= 11.8 and released >= 14.0,
                     f'exit+0.5..2.5m max={held:.1f} km/h (arc 10.8)  '
                     f'exit+10m={released:.1f} km/h'))

    # ── Mission speed ahead (speed.anticipate_mission) ───────────────────────
    # From x = 30 m on the waypoints belong to mission state 1 (8 km/h); the car
    # is still in state 0 (15 km/h) at 15 km/h. It must be braking before the
    # boundary rather than after it: 4 m out the backward pass (a_dec 1.5,
    # 0.55 s lag lookahead) allows ~11 km/h, and 25 m out it is unaffected.
    slow_ahead = straight_path()
    for w in slow_ahead:
        if w.pose.pose.position.x >= 30.0:
            w.mission_state = 1
    # With mission.uniform_kmh (or equal state speeds) there is no slower state
    # to brake for, so the check does not apply.
    uniform, state_kmh = node_params(h, ['mission.uniform_kmh', 'mission.state_kmh'])
    table = [15, 8]
    s0 = uniform if uniform is not None and uniform >= 0 else (
        state_kmh[0] if state_kmh and state_kmh[0] >= 0 else table[0])
    s1 = uniform if uniform is not None and uniform >= 0 else (
        state_kmh[1] if state_kmh and state_kmh[1] >= 0 else table[1])
    if s1 < s0 - 1.0:
        c_far2 = h.feed(slow_ahead, x=5.0, y=0.0, yaw=0.0, speed_mps=15 / 3.6, duration=2.5)
        c_near = h.feed(slow_ahead, x=26.0, y=0.0, yaw=0.0, speed_mps=15 / 3.6, duration=1.5)
        res.append(check('slower mission state ahead is braked for early',
                         c_far2.speed * 3.6 > min(s0, 15) - 0.5 and c_near.speed * 3.6 < 13.0,
                         f'25 m out={c_far2.speed * 3.6:.1f} km/h  4 m out={c_near.speed * 3.6:.1f} km/h '
                         f'(state 1 = {s1:g})'))
    else:
        print(f'[SKIP] {"slower mission state ahead is braked for early":<46} '
              f'state 0 and 1 both {s0:g} km/h (uniform speed)')

    # ── Hunting detector at speed (osc.*) ────────────────────────────────────
    # Fed at 25 km/h on the line with the wheels following the command. Nothing
    # before this point closed the steering loop, so the 8 s history is clean.
    v25 = 25 / 3.6
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=v25, duration=2.0, wheels_follow_cmd=True)
    v_calm25 = h.cmd.speed * 3.6

    # Localisation-style jitter: heading noise, 1 deg std every 40 ms. The raw
    # command then swings past 0.5 m/s^2 many times a second; the low-pass on
    # the lateral acceleration must keep it from reading as hunting.
    rng = np.random.default_rng(7)
    end = time.time() + 7.0
    while time.time() < end:
        h.feed(sp, x=10.0, y=0.0, yaw=math.radians(rng.normal(0.0, 1.0)),
               speed_mps=v25, duration=0.04, wheels_follow_cmd=True)
    v_jit = h.cmd.speed * 3.6
    res.append(check('steering jitter at 25 km/h is not hunting',
                     v_jit >= v_calm25 - 0.1,
                     f'calm={v_calm25:.1f} km/h  jitter={v_jit:.1f} km/h'))

    # A slow weave, 0.4 Hz and +-3 deg of heading. At 25 km/h that is a few
    # m/s^2 of lateral acceleration, but the old 2 s window saw at most two
    # flips of it. The front axle stays within the cross-track deadband, so a
    # cut here comes from the detector alone.
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=v25, duration=8.5, wheels_follow_cmd=True)
    t0 = time.time()
    while time.time() - t0 < 7.5:
        yaw = math.radians(3.0) * math.sin(2.0 * math.pi * 0.4 * (time.time() - t0))
        h.feed(sp, x=10.0, y=0.0, yaw=yaw, speed_mps=v25, duration=0.04,
               wheels_follow_cmd=True)
    v_weave = h.cmd.speed * 3.6
    res.append(check('slow weave at 25 km/h cuts speed',
                     v_weave < v_calm25 * 0.95,
                     f'calm={v_calm25:.1f} km/h  0.4 Hz weave={v_weave:.1f} km/h'))

    # ── Curvature-linked lookahead cap (lookahead.curv_*) ────────────────────
    # Walk through the 3 m radius arc at 3 m/s. The speed schedule alone asks
    # for 1.6*3+4 = 8.8 m; the cap is 2.3*sqrt(3) = 4.0 m inside the arc, must
    # still hold just past the exit (curv_hold_m 5), and must have let go 10 m
    # later. Only meaningful with the cap on and no per-state cap in the way.
    coef, hold, caps = node_params(h, ['lookahead.curv_coef', 'lookahead.curv_hold_m',
                                       'lookahead.state_cap_m'])
    if coef and coef > 0 and caps and caps[0] == 0.0:
        lds = []
        s = s_exit - 12.0
        while s < s_exit + 12.0:
            x, y, yaw = pose_at(arc, s)
            h.feed(arc, x=x, y=y, yaw=yaw, speed_mps=3.0, duration=0.1)
            lds.append((s - s_exit, h.ld))
            s += 0.3
        want = max(3.0, coef * math.sqrt(3.0))
        in_arc = max(ld for d, ld in lds if -3.0 <= d <= 0.0 and ld is not None)
        just_out = max(ld for d, ld in lds if 0.5 <= d <= min(2.5, hold - 1.0) and ld is not None)
        later = min(ld for d, ld in lds if d >= 10.0 and ld is not None)
        if rows is not None:
            # Real course: from a straight (k ~0.015, not 0) onto the
            # approach to the R4.6 bend. The cap must start coming down at once.
            # A cap left above lookahead.max_m kept it at 11 m for ~2 s here.
            for _ in range(8):
                h.feed(course_window(rows, 2005), x=rows[2010][1], y=rows[2010][2],
                       yaw=rows[2010][4], speed_mps=6.9, duration=0.2, mission=6)
            t_arr = time.time()
            t_short = None
            while time.time() - t_arr < 3.0:
                h.feed(course_window(rows, 645), x=rows[650][1], y=rows[650][2],
                       yaw=rows[650][4], speed_mps=5.0, duration=0.1, mission=1)
                if t_short is None and h.ld is not None and h.ld < 10.5:
                    t_short = time.time() - t_arr
            res.append(check('lookahead starts shortening at once for a bend',
                             t_short is not None and t_short < 0.6 and h.ld < 5.5,
                             f'below 10.5 m after {t_short if t_short is None else round(t_short, 2)} s, '
                             f'{h.ld:.1f} m after 3 s'))
        res.append(check('lookahead shortens for a bend and holds past it',
                         in_arc <= want + 0.3 and just_out <= want + 0.5 and later >= 7.0,
                         f'in arc {in_arc:.1f} m, exit+0.5..2.5 m {just_out:.1f} m, '
                         f'exit+10 m {later:.1f} m (cap {want:.1f}, schedule 8.8)'))
    else:
        print(f'[SKIP] {"lookahead shortens for a bend and holds past it":<46} '
              f'curv_coef={coef} state_cap_m[0]={caps[0] if caps else None}')

    # ACC: acc_index is the local-path index nearest the obstacle ahead. Over
    # 200 keeps the speed, 100-200 slows on a sigmoid, and <= 100 stops, 0 and
    # negative included (an obstacle at the start of the local path is at the car).
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=2.0)
    acc_res = {}
    for idx in (250, 150, 50, 0, -1):
        c = h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=1.0, acc=(True, idx))
        acc_res[idx] = (c.speed * 3.6, c.brake)
    h.feed(sp, x=10.0, y=0.0, yaw=0.0, speed_mps=3.0, duration=0.5, acc=(False, 0))
    stops = [acc_res[i][0] == 0.0 and acc_res[i][1] >= 90 for i in (50, 0, -1)]
    res.append(check('ACC index <= 100 (0, negative too) stops',
                     acc_res[250][0] > 1.0 and acc_res[150][0] > 0.0 and all(stops),
                     '  '.join(f'{i}:{v:.1f}km/h b{b}' for i, (v, b) in acc_res.items())))

    # /odom must be best effort (waypointFollower.h): a RELIABLE subscription
    # drained by the 20 Hz loop fell ~250 ms behind for whole runs in simulation.
    # The failure itself is random, so check the setting instead.
    infos = [i for i in h.get_subscriptions_info_by_topic('/odom')
             if i.node_name == 'waypoint_follower']
    rel = [i.qos_profile.reliability for i in infos]
    res.append(check('/odom subscription is best effort',
                     bool(rel) and all(r == ReliabilityPolicy.BEST_EFFORT for r in rel),
                     f'{[r.name for r in rel]}'))

    print(f'\n{sum(res)}/{len(res)} passed')
    rclpy.shutdown()
    return 0 if all(res) else 1


if __name__ == '__main__':
    raise SystemExit(main())
