#!/usr/bin/env python3
"""Identify the ERP42 Racing plant and the tracking quality from a rosbag2 recording.

    python3 waypoint_follower/test/fit_from_bag.py <bag_dir> [--wheelbase 1.212]

Record with waypoint_follower/test/record_fit_bag.sh (auto mode, the normal
driving stack running): a few laps at the current speeds, one stop driven by the
stack (for the braking decel), both 0.csv and 1.csv if possible. The numbers
below were checked against sim bags with a known plant (steering tau, gain and
offset, speed tau and accel/decel limits, position/yaw lag, wheelbase: all within
a few percent). A rate limit that a lap never reaches is reported as a lower
bound only; a steering step test is needed to pin it down.

What it estimates, and where each number goes:

  steering actuator  dead time, first-order lag, rate limit, gain, offset
                     -> sim plant (steer_lag_tau, steer_rate_limit_dps, steer_bias_deg)
                     -> serial steering_offset_deg (alignment)
  speed loop         dead time, first-order lag, accel / decel (no brake), brake decel,
                     steady-state gain (MPS2BYTE check), wheel-speed vs GPS scale
                     -> sim plant (speed_lag_tau, max_accel_mps2)
                     -> tracking_params speed.lag_comp_sec, speed.a_dec_max, a_acc_max
                     -> aeb decel_capability_mps2
  localisation       /odom position lag and yaw lag relative to the wheel feedback,
                     odom rate, yaw lag against raw /imu/data
                     -> tracking_params delay_comp_sec, odom_information yaw_lpf_tau_sec
  vehicle            effective wheelbase (yaw rate vs v*tan(delta)), understeer trend
  tracking           signed / absolute CTE against /final_waypoint per speed band and
                     mission state, steering hunting, lateral acceleration

Conventions: ControlCommand.steering is left-positive; Feedback.steering comes back
left-NEGATIVE (serial_bridge RX), so it is negated here.
"""
import argparse
import datetime
import math
import os
import re
import sys

import numpy as np
from scipy import optimize, signal

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DT = 0.02   # resampling step [s]

# ── --apply targets ──────────────────────────────────────────────────────────
# Only the plant/localisation numbers that are pure physical measurements go
# here (tracking_params.yaml's speed/delay fields, the serial alignment offset,
# the heading low-pass). Stanley gains are a closed-loop control law, not a
# plant parameter: a regression against a bag recorded under the CURRENT gains
# would just recover those same gains, so they are never auto-applied. AEB's
# decel_capability_mps2 and WHEEL_BASE are safety- and geometry-sensitive and
# are left for manual review even though fit_from_bag.py suggests values for
# them; the sim plant lives outside this repo (~/wf_sim_scratch) and is edited
# by hand for the same reason.
_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.abspath(os.path.join(_HERE, '..', '..'))
APPLY_TARGETS = {
    ('tracking_params', 'delay_comp_sec'):
        (os.path.join(_HERE, '..', 'config', 'tracking_params.yaml'), 'yaml'),
    ('tracking_params', 'speed.a_dec_max'):
        (os.path.join(_HERE, '..', 'config', 'tracking_params.yaml'), 'yaml'),
    ('tracking_params', 'speed.lag_comp_sec'):
        (os.path.join(_HERE, '..', 'config', 'tracking_params.yaml'), 'yaml'),
    ('serial_bridge', 'steering_offset_deg'):
        (os.path.join(_WS_SRC, 'erp42_racing_ros-jazzy-harmonic', 'src',
                      'erp42_racing_serial', 'config', 'serial_bridge.param.yaml'), 'yaml'),
    ('odom_information', 'gps_imu.yaw_lpf_tau_sec'):
        (os.path.join(_WS_SRC, 'odom_information', 'launch', 'odom_information.launch.py'), 'launch_py'),
}


def apply_suggestions(sug):
    """Writes the physical-measurement suggestions into their live config files.
    Backs up each touched file once (timestamped .bak) before the first edit."""
    backed_up = set()
    applied, skipped = [], []
    for tgt, name, val, why in sug:
        key = (tgt, name)
        if key not in APPLY_TARGETS:
            skipped.append((tgt, name, val, 'not auto-applied: needs review (closed-loop gain, '
                            'safety parameter, or lives outside this repo)'))
            continue
        path, kind = APPLY_TARGETS[key]
        path = os.path.normpath(path)
        if not os.path.exists(path):
            skipped.append((tgt, name, val, f'file not found: {path}'))
            continue
        text = open(path).read()
        bare_key = name.split('.')[-1]
        if kind == 'yaml':
            pattern = re.compile(rf'^(\s*{re.escape(bare_key)}\s*:\s*)([+-]?[\d.]+)', re.MULTILINE)
        else:  # launch_py: a quoted dotted key used as a flat launch parameter
            pattern = re.compile(rf"('{re.escape(name)}'\s*:\s*)([+-]?[\d.]+)")
        m = pattern.search(text)
        if not m:
            skipped.append((tgt, name, val, f'key not found as expected in {path}'))
            continue
        old_val = m.group(2)
        if old_val == val:
            skipped.append((tgt, name, val, 'already at this value'))
            continue
        if path not in backed_up:
            bak = f'{path}.bak.{datetime.datetime.now():%Y%m%d_%H%M%S}'
            open(bak, 'w').write(text)
            backed_up.add(path)
        new_text = pattern.sub(lambda mm: mm.group(1) + val, text, count=1)
        open(path, 'w').write(new_text)
        applied.append((tgt, name, old_val, val, path))
    return applied, skipped, backed_up


# ── bag reading ──────────────────────────────────────────────────────────────
def storage_id(path):
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.exists(meta):
        for line in open(meta):
            if 'storage_identifier' in line:
                return line.split(':', 1)[1].strip()
    return 'sqlite3'


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id(path)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = ['/erp42_racing/control_command', '/erp42_racing/feedback', '/odom', '/imu/data',
            '/final_waypoint', '/gps_state', '/pva/ins_status']
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}
    d = {k: [] for k in ('cmd', 'fb', 'odom', 'imu', 'path', 'state')}
    while reader.has_next():
        topic, raw, t = reader.read_next()
        t *= 1e-9
        m = deserialize_message(raw, cls[topic])
        if topic == '/erp42_racing/control_command':
            d['cmd'].append((t, m.speed, m.steering, m.brake))
        elif topic == '/erp42_racing/feedback':
            d['fb'].append((t, m.speed, -m.steering, m.brake, float(m.manual_mode), float(m.emergency_stop)))
        elif topic == '/odom':
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            d['odom'].append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        elif topic == '/imu/data':
            q = m.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            d['imu'].append((t, yaw, m.angular_velocity.z))
        elif topic == '/final_waypoint':
            if m.waypoints:
                d['path'].append((t, np.array([[w.pose.pose.position.x, w.pose.pose.position.y]
                                               for w in m.waypoints])))
        elif topic == '/gps_state':
            d['state'].append((t, m.current_state, m.lane_number))
    for k in ('cmd', 'fb', 'odom', 'imu', 'state'):
        d[k] = np.array(d[k]) if d[k] else np.zeros((0, 7))
    return d, types


def rate(ts):
    return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 else float('nan')


def zoh(t_src, v_src, t):
    i = np.searchsorted(t_src, t, side='right') - 1
    return v_src[np.clip(i, 0, len(v_src) - 1)]


def segments(mask, min_len):
    """Index ranges [a, b) where mask is True for at least min_len samples."""
    out, a = [], None
    for i, m in enumerate(mask):
        if m and a is None:
            a = i
        elif not m and a is not None:
            if i - a >= min_len:
                out.append((a, i))
            a = None
    if a is not None and len(mask) - a >= min_len:
        out.append((a, len(mask)))
    return out


def xcorr_lag(a, b, max_lag_s):
    """Lag (s) by which b trails a; both on the DT grid, NaNs dropped as zeros."""
    a = np.nan_to_num(a - np.nanmean(a)); b = np.nan_to_num(b - np.nanmean(b))
    n = int(max_lag_s / DT)
    best, lag = -1e18, 0
    for k in range(-n, n + 1):
        c = np.dot(a[max(0, -k):len(a) - max(0, k)], b[max(0, k):len(b) - max(0, -k)])
        if c > best:
            best, lag = c, k
    # parabolic refinement
    return lag * DT


# ── first-order-plus-dead-time fit (linear part; rate limit estimated apart) ──
def fopdt_cost(params, t_grid, u, y, segs, fixed_gain=None):
    d, tau, g, b = params[0], params[1], (params[2] if fixed_gain is None else fixed_gain), params[-1]
    if tau <= 0.005 or d < 0:
        return 1e9
    a = math.exp(-DT / tau)
    k = int(round(d / DT))
    err = []
    for s0, s1 in segs:
        if s1 - s0 <= k + 5:
            continue
        x = g * u[s0:s1 - k] + b
        yy = y[s0 + k:s1]
        zi = signal.lfiltic([0, 1 - a], [1, -a], [yy[0]], [x[0]])
        pred = signal.lfilter([0, 1 - a], [1, -a], x, zi=zi)[0]
        err.append(pred - yy)
    if not err:
        return 1e9
    e = np.concatenate(err)
    return float(np.sqrt(np.mean(e * e)))


def fit_fopdt(t_grid, u, y, segs, with_gain=True):
    best = None
    for d in np.arange(0.0, 0.42, 0.02):
        for tau0 in (0.1, 0.3, 0.6):
            x0 = [d, tau0, 1.0, 0.0] if with_gain else [d, tau0, 0.0]
            f = (lambda p, d=d: fopdt_cost([d] + list(p), t_grid, u, y, segs, None if with_gain else 1.0))
            r = optimize.minimize(f, x0[1:], method='Nelder-Mead',
                                  options={'xatol': 1e-3, 'fatol': 1e-5, 'maxiter': 400})
            if best is None or r.fun < best[0]:
                best = (r.fun, d, *r.x)
    return best


def grid_fit(u, y, segs, D, TAU, UP, DN, G, B=(0.0,), clamp=None, sym=False):
    """Grid search of y' = clip((g*u(t-d)+b - y)/tau, -dn, +up), simulated on each segment.

    Same structure as the sim plant (first-order lag, then a slew limit per step).
    Returns (cost, d, tau, up, dn, g, b) of the best candidate and the per-candidate
    cost/parameter arrays for bound checks.
    """
    import itertools
    if sym:     # one rate limit for both directions (steering actuator)
        cand = np.array([(d, t, r, r, g, b) for d, t, r, g, b in itertools.product(D, TAU, UP, G, B)], float)
    else:
        cand = np.array(list(itertools.product(D, TAU, UP, DN, G, B)), float)
    dk = np.round(cand[:, 0] / DT).astype(int)
    alpha = 1.0 - np.exp(-DT / cand[:, 1])
    up, dn, g, b = cand[:, 2] * DT, cand[:, 3] * DT, cand[:, 4], cand[:, 5]
    kmax = dk.max()
    sse = np.zeros(len(cand)); n = 0
    for s0, s1 in segs:
        if s1 - s0 < kmax + 10:
            continue
        state = np.full(len(cand), y[s0 + kmax])
        for k in range(s0 + kmax, s1 - 1):
            x = g * u[k - dk] + b
            step = np.clip(alpha * (x - state), -dn, up)
            state = state + step
            if clamp is not None:
                state = np.clip(state, -clamp, clamp)
            e = state - y[k + 1]
            sse += e * e
        n += s1 - 1 - (s0 + kmax)
    cost = np.sqrt(sse / max(n, 1))
    i = int(np.argmin(cost))
    return cost, cand, i


def bound_note(cost, cand, i, col, grid, scale=1.0):
    """Is the chosen limit actually identified? Near-optimal candidates (cost within
    2 %) spanning up to the top of the grid mean the limit was never reached."""
    near = cost <= cost[i] * 1.02
    vals = cand[near, col]
    if vals.max() >= max(grid) - 1e-9 and vals.min() < max(grid):
        return f'not reached in this log (>= {vals.min() * scale:.2f})', vals.min(), False
    return '', cand[i, col], True


# ── main analysis ────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--wheelbase', type=float, default=1.212)
    ap.add_argument('--apply', action='store_true',
                    help='write the physical-measurement suggestions (tracking_params.yaml '
                         'delay_comp_sec/speed.a_dec_max/speed.lag_comp_sec, serial_bridge '
                         'steering_offset_deg, odom_information gps_imu.yaw_lpf_tau_sec) into their '
                         'live config files. Everything else (Stanley gains, AEB decel capability, '
                         'WHEEL_BASE, the sim plant) still needs a human to read the printed table.')
    args = ap.parse_args()
    L = args.wheelbase

    d, types = read_bag(args.bag)
    cmd, fb, od, imu = d['cmd'], d['fb'], d['odom'], d['imu']
    print(f'== {args.bag}')
    for k, arr in (('control_command', cmd), ('feedback', fb), ('odom', od), ('imu/data', imu)):
        print(f'   {k:16s} {len(arr):6d} msgs  {rate(arr[:, 0]) if len(arr) else float("nan"):6.1f} Hz')
    print(f'   final_waypoint   {len(d["path"]):6d} msgs')
    if len(cmd) < 50 or len(fb) < 50 or len(od) < 50:
        sys.exit('need control_command, feedback and odom')

    t0 = max(cmd[0, 0], fb[0, 0], od[0, 0]); t1 = min(cmd[-1, 0], fb[-1, 0], od[-1, 0])
    t = np.arange(t0, t1, DT)
    u_st = zoh(cmd[:, 0], cmd[:, 2], t)                    # rad, left +
    u_v = zoh(cmd[:, 0], cmd[:, 1], t)                     # m/s
    u_br = zoh(cmd[:, 0], cmd[:, 3], t)
    y_st = np.interp(t, fb[:, 0], fb[:, 2])                # rad, left +
    y_v = np.interp(t, fb[:, 0], fb[:, 1])
    manual = zoh(fb[:, 0], fb[:, 4], t) > 0.5
    estop = zoh(fb[:, 0], fb[:, 5], t) > 0.5
    auto = ~manual & ~estop
    oyaw = np.unwrap(od[:, 3])
    ox = np.interp(t, od[:, 0], od[:, 1]); oy = np.interp(t, od[:, 0], od[:, 2])
    oyaw_t = np.interp(t, od[:, 0], oyaw)
    print(f'   duration {t1 - t0:.0f} s, auto {auto.mean() * 100:.0f} %, '
          f'max speed {y_v.max() * 3.6:.1f} km/h')

    # derived odom signals (smoothed over 0.2 s so GPS noise does not dominate)
    w = int(0.2 / DT)
    k = np.ones(w) / w
    v_odom = np.hypot(np.gradient(np.convolve(ox, k, 'same'), DT), np.gradient(np.convolve(oy, k, 'same'), DT))
    r_odom = np.gradient(np.convolve(oyaw_t, k, 'same'), DT)
    r_model = y_v * np.tan(y_st) / L
    edge = slice(w, len(t) - w)

    rec = {}
    print('\n── steering actuator (cmd -> feedback) ──')
    seg_st = segments(auto, int(3 / DT))
    fs = fit_fopdt(t, u_st, y_st, seg_st)
    rmse, dd, tau, g, b = fs
    RATES = [10, 15, 20, 30, 45, 60, 90, 120, 180, 240, 360]
    cost, cand, i = grid_fit(u_st, y_st, seg_st, D=np.arange(0, 0.22, 0.02),
                             TAU=np.geomspace(0.05, 0.8, 16), UP=np.radians(RATES), DN=None,
                             G=np.arange(0.9, 1.101, 0.025), B=[b], clamp=math.radians(20), sym=True)
    dd, tau, g = cand[i, 0], cand[i, 1], cand[i, 4]
    note, rate_v, ok = bound_note(cost, cand, i, 2, np.radians(RATES), scale=180 / math.pi)
    rate_obs = math.degrees(rate_v)
    print(f'   dead time {dd:.2f} s, lag tau {tau:.2f} s, gain {g:.3f}, offset {math.degrees(b):+.2f} deg '
          f'(rmse {math.degrees(cost[i]):.2f} deg)')
    print(f'   rate limit {"~" + format(rate_obs, ".0f") if ok else "unknown"} deg/s {note}')
    rec.update(steer_dead=dd, steer_tau=tau, steer_gain=g, steer_off=math.degrees(b),
               steer_rate=rate_obs if ok else float('nan'), steer_rate_lb=rate_obs)

    print('\n── speed loop (cmd -> wheel speed) ──')
    run = auto & (u_br <= 1)
    seg_v = segments(run, int(3 / DT))
    ACC = [0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0]
    cost, cand, i = grid_fit(u_v, y_v, seg_v, D=np.arange(0, 0.32, 0.04), TAU=np.geomspace(0.1, 1.5, 14),
                             UP=ACC, DN=ACC, G=np.arange(0.9, 1.101, 0.05))
    dv, tauv, gv = cand[i, 0], cand[i, 1], cand[i, 4]
    note_a, a_acc, ok_a = bound_note(cost, cand, i, 2, ACC)
    note_d, a_dec, ok_d = bound_note(cost, cand, i, 3, ACC)
    vdot = np.gradient(np.convolve(y_v, np.ones(5) / 5, 'same'), DT)
    brk = auto & (u_br >= 90) & (y_v > 0.5)
    a_brk = -np.median(vdot[brk]) if brk.sum() > 10 else float('nan')
    print(f'   dead time {dv:.2f} s, lag tau {tauv:.2f} s, steady gain {gv:.3f} (1.0 = MPS2BYTE right), '
          f'rmse {cost[i]:.3f} m/s')
    print(f'   accel limit {a_acc:.2f} m/s^2 {note_a}')
    print(f'   decel on the speed command alone {a_dec:.2f} m/s^2 {note_d}')
    print(f'   decel with brake >= 90 (median) {a_brk:.2f} m/s^2  [{brk.sum()} samples'
          f'{"; stop the car with the stack once to measure it" if brk.sum() <= 10 else ""}]')
    mv = auto & (y_v > 1.5) & (np.abs(np.gradient(y_v, DT)) < 0.3)
    scale = np.median(v_odom[edge][mv[edge]] / y_v[edge][mv[edge]]) if mv.sum() > 50 else float('nan')
    print(f'   GPS speed / wheel speed (steady) {scale:.3f}  (1.0 = BYTE2MPS / wheel radius right)')
    rec.update(v_dead=dv, v_tau=tauv, v_gain=gv, a_acc=a_acc, a_dec=a_dec, a_brk=a_brk, v_scale=scale, a_dec_ok=ok_d)

    print('\n── localisation lag (relative to wheel feedback) ──')
    moving = auto & (y_v > 1.0)
    # v_wheel - v_odom = lag * a + (1 - scale) * v  (a GPS position that is `lag` late
    # reads a speed that is `lag` late, i.e. short by lag*a while accelerating)
    ws = int(0.5 / DT); kk = np.ones(ws) / ws
    vo = np.convolve(v_odom, kk, 'same'); vf = np.convolve(y_v, kk, 'same')
    af = np.gradient(vf, DT)
    selp = moving & (np.abs(af) > 0.3); selp[:ws] = False; selp[-ws:] = False
    if selp.sum() > 50:
        Xr = np.column_stack([af[selp], vf[selp]])
        coef = np.linalg.lstsq(Xr, vf[selp] - vo[selp], rcond=None)[0]
        lag_pos = float(coef[0])
    else:
        lag_pos = float('nan')
    lag_yaw = xcorr_lag(np.where(moving, r_model, 0)[edge], np.where(moving, r_odom, 0)[edge], 0.6)
    print(f'   /odom position trails wheel speed by {lag_pos:.2f} s; /odom yaw trails v*tan(delta)/L '
          f'by {lag_yaw:.2f} s')
    print(f'   /odom rate {rate(od[:, 0]):.1f} Hz')
    if len(imu) > 50:
        r_imu = np.interp(t, imu[:, 0], imu[:, 2])
        yaw_imu = np.interp(t, imu[:, 0], np.unwrap(imu[:, 1]))
        lag_imu = xcorr_lag(np.where(moving, r_imu, 0)[edge], np.where(moving, r_odom, 0)[edge], 0.8)
        lag_imu_model = xcorr_lag(np.where(moving, r_model, 0)[edge], np.where(moving, r_imu, 0)[edge], 0.6)
        print(f'   /odom yaw trails raw /imu/data by {lag_imu:.2f} s (odom_information gps_imu.yaw_lpf_tau_sec); '
              f'IMU trails the wheel model by {lag_imu_model:.2f} s')
        rec.update(lag_imu=lag_imu)
    rec.update(lag_pos=lag_pos, lag_yaw=lag_yaw)

    print('\n── vehicle: yaw rate vs v*tan(delta) ──')
    ks = int(round(lag_yaw / DT))
    rm = r_model[:len(t) - ks] if ks > 0 else r_model
    ro = r_odom[ks:] if ks > 0 else r_odom
    vv = y_v[:len(rm)]; st = y_st[:len(rm)]; au = auto[:len(rm)]
    sel = au & (vv > 1.5) & (np.abs(st) > math.radians(2))
    sel[:w] = False; sel[-w:] = False
    if sel.sum() > 50:
        xv = vv[sel] * np.tan(st[sel]); yr = ro[sel]
        slope = np.dot(xv, yr) / np.dot(xv, xv)
        L_eff = 1.0 / slope if slope > 0 else float('nan')
        alat = np.abs(vv[sel] * yr)
        bins = [(0, 1), (1, 2), (2, 3), (3, 10)]
        trend = []
        for lo, hi in bins:
            m = (alat >= lo) & (alat < hi)
            if m.sum() > 30:
                s = np.dot(xv[m], yr[m]) / np.dot(xv[m], xv[m])
                trend.append(f'{lo}-{hi} m/s^2: {1 / s:.3f} m')
        print(f'   effective wheelbase {L_eff:.3f} m (nominal {L})   by lateral accel: ' + ', '.join(trend))
        rec.update(L_eff=L_eff)
    straight = auto & (y_v > 2.0) & (np.abs(r_odom) < 0.01)
    if straight.sum() > 50:
        c0 = math.degrees(np.median(u_st[straight]))
        f0 = math.degrees(np.median(y_st[straight]))
        print(f'   driving straight ({straight.sum()} samples): command {c0:+.2f} deg, wheel reading {f0:+.2f} deg '
              f'(left +). A non-zero command is the alignment error the controller is carrying.')
        rec.update(straight_cmd=c0)

    print('\n── tracking against /final_waypoint ──')
    if d['path']:
        pt = np.array([p[0] for p in d['path']])
        st_arr = d['state']
        cte = np.full(len(t), np.nan)
        for i in range(0, len(t), 2):
            j = np.searchsorted(pt, t[i]) - 1
            if j < 0:
                continue
            P = d['path'][j][1]
            A, B = P[:-1], P[1:]
            AB = B - A; ln2 = (AB ** 2).sum(1)
            ok = ln2 > 1e-12
            if not ok.any():
                continue
            p = np.array([ox[i], oy[i]])
            u = np.clip(((p - A) * AB).sum(1) / np.where(ok, ln2, 1), 0, 1)
            Q = A + AB * u[:, None]
            dist = np.hypot(*(p - Q).T); dist[~ok] = np.inf
            m = int(np.argmin(dist))
            tx, ty = AB[m] / math.sqrt(ln2[m])
            cte[i] = tx * (p[1] - Q[m, 1]) - ty * (p[0] - Q[m, 0])      # + = left of path
        valid = ~np.isnan(cte) & auto & (y_v > 0.5)
        vk = y_v * 3.6
        print(f'   {"speed band":12s} {"n":>6s} {"mean(+L)":>8s} {"p95|e|":>7s} {"max|e|":>7s} {"steer hf std":>12s}')
        hf = np.degrees(y_st - np.convolve(y_st, np.ones(50) / 50, 'same'))
        for lo, hi in ((0, 10), (10, 15), (15, 20), (20, 26)):
            m = valid & (vk >= lo) & (vk < hi)
            if m.sum() < 25:
                continue
            e = cte[m]
            print(f'   {lo:2d}-{hi:2d} km/h   {m.sum():6d} {np.mean(e):+8.3f} {np.percentile(np.abs(e), 95):7.3f} '
                  f'{np.abs(e).max():7.3f} {np.std(hf[m]):12.2f}')
        if len(st_arr):
            state_t = zoh(st_arr[:, 0], st_arr[:, 1], t)
            worst = []
            for s in np.unique(state_t[valid]).astype(int):
                m = valid & (state_t == s)
                worst.append((np.abs(cte[m]).max(), s, np.percentile(np.abs(cte[m]), 95), vk[m].max()))
            worst.sort(reverse=True)
            print('   worst mission states: ' + '; '.join(
                f'state {s}: max {mx:.2f} m, p95 {p9:.2f} m, up to {v:.0f} km/h' for mx, s, p9, v in worst[:4]))
        rec.update(cte_max=np.nanmax(np.abs(cte[valid])) if valid.any() else float('nan'),
                   cte_mean=np.nanmean(cte[valid]) if valid.any() else float('nan'))
        a_lat = np.abs(y_v * np.convolve(r_odom, np.ones(10) / 10, 'same'))
        m = auto & (y_v > 1); m[:50] = False; m[-50:] = False
        print(f'   lateral acceleration p99 {np.percentile(a_lat[m], 99):.2f} m/s^2, '
              f'p99.9 {np.percentile(a_lat[m], 99.9):.2f}')

    print('\n── suggested values (check before use) ──')
    sug = []
    sug.append(('sim fake_erp42', 'steer_lag_tau', f'{rec["steer_tau"] + rec["steer_dead"]:.2f}',
                'lag + dead time (the sim plant has no separate dead time)'))
    if not math.isnan(rec['steer_rate']):
        sug.append(('sim fake_erp42', 'steer_rate_limit_dps', f'{rec["steer_rate"]:.0f}', 'identified'))
    else:
        sug.append(('sim fake_erp42', 'steer_rate_limit_dps', f'>={rec["steer_rate_lb"]:.0f}',
                    'not reached: do a steering step test to pin it down'))
    if abs(rec['steer_off']) > 0.1:
        sug.append(('sim fake_erp42', 'steer_bias_deg', f'{rec["steer_off"]:+.2f}',
                    'wheels sit this far from the command'))
    sug.append(('sim fake_erp42', 'speed_lag_tau', f'{rec["v_tau"] + rec["v_dead"]:.2f}', ''))
    sug.append(('sim fake_erp42', 'max_accel_mps2', f'{min(rec["a_acc"], rec["a_dec"]):.2f}',
                'the sim uses one limit for both directions'))
    sug.append(('tracking_params', 'speed.a_dec_max', f'{0.8 * rec["a_dec"]:.2f}',
                '80 % of the decel the car reaches on the speed command alone'
                + ('' if rec['a_dec_ok'] else ' (lower bound: never saturated here)')))
    sug.append(('tracking_params', 'speed.lag_comp_sec', f'{rec["v_dead"] + rec["v_tau"] + 0.1:.2f}',
                'dead time + lag + 0.1, then re-check in sim'))
    sug.append(('tracking_params', 'delay_comp_sec', f'{max(0.0, rec["lag_pos"]):.2f}',
                'position lag; yaw lag reported separately'))
    if 'straight_cmd' in rec and abs(rec['straight_cmd']) > 0.2:
        sug.append(('serial_bridge', 'steering_offset_deg', f'{rec["straight_cmd"]:+.2f}',
                    'only if repeatable across runs (RX sign fixed 2026-09-27, feedback stays unbiased)'))
    if 'lag_imu' in rec and rec['lag_imu'] > 0.12:
        sug.append(('odom_information', 'gps_imu.yaw_lpf_tau_sec', f'{max(0.03, rec["lag_imu"] - 0.05):.2f}',
                    'heading lag over IMU is large; lower only if yaw stays steady at standstill'))
    if not math.isnan(rec['a_brk']):
        sug.append(('aeb_params', 'decel_capability_mps2', f'{0.8 * rec["a_brk"]:.2f}',
                    '80 % of the braking decel measured (brake >= 90)'))
    if 'L_eff' in rec and abs(rec['L_eff'] - L) / L > 0.05:
        sug.append(('waypoint_follower', 'WHEEL_BASE', f'{rec["L_eff"]:.3f}',
                    'effective value differs >5 %: check steering gain / understeer first'))
    for tgt, name, val, why in sug:
        print(f'   {tgt:16s} {name:24s} {val:>7s}   {why}')

    if args.apply:
        print('\n── --apply ──')
        applied, skipped, backed_up = apply_suggestions(sug)
        for path in sorted(backed_up):
            print(f'   backed up {path}')
        for tgt, name, old_val, val, path in applied:
            print(f'   WROTE  {tgt:16s} {name:24s} {old_val} -> {val}   ({path})')
        for tgt, name, val, why in skipped:
            print(f'   skip   {tgt:16s} {name:24s} {val:>7s}   {why}')
        if applied:
            print('\n   Rebuild/relaunch the touched packages to pick these up '
                  '(tracking_params.yaml and serial_bridge.param.yaml are read at node '
                  'startup; odom_information.launch.py takes effect on the next launch).')
        else:
            print('\n   Nothing written.')


if __name__ == '__main__':
    main()
