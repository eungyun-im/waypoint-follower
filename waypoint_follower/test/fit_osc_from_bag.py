#!/usr/bin/env python3
"""Steering oscillation per speed band, from bags recorded with record_fit_bag.sh.

    python3 waypoint_follower/test/fit_osc_from_bag.py <bag_dir> [<bag_dir> ...]

Reads /wf_debug/terms (every term of the steering law, one message per control
cycle) and prints, per speed band:

  cte rms / p95    tracking error [m]
  weave Hz / amp   dominant frequency of the cross-track error in 0.1-2 Hz and
                   its amplitude [m]; this is the oscillation being tuned out
  cmd hf           steering command in that same 0.1-2 Hz band, std [deg]. A
                   corner entry also lands in this band, so compare like with
                   like (same course, same speed)
  pp / head / cross / rate / lag
                   how much each term puts into that band, std [deg], already
                   weighted by the pure-pursuit / Stanley blend. The largest
                   one is the term whose gain to change first.
  fb lag           measured wheel angle behind the command [s]

Fitting is a comparison between runs: change one gain, record again, and pass
both bags. Each bag's own gains (wf_params.yaml, saved by record_fit_bag.sh)
are printed next to its numbers.

plant numbers (delays, wheelbase, braking) come from fit_from_bag.py, and
should be fixed first: see TUNING_NOTES.md section 4.5.
"""
import argparse
import os
import sys

import numpy as np
from scipy import signal
import yaml

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Index in /wf_debug/terms; keep in step with DbgTerm in waypointFollower.h.
TERMS = ['v_mps', 'ratio', 'pp_deg', 'st_deg', 'blend_deg', 'cmd_deg',
         'heading_deg', 'cross_deg', 'rate_deg', 'lag_deg', 'cte_m',
         'ld_m', 'ld_cap_m', 'curv_limit_kmh', 'cte_factor', 'osc_factor',
         'cmd_kmh', 'fb_steer_deg', 'mission_state']
BANDS = ((0, 5), (5, 10), (10, 15), (15, 20), (20, 30))   # km/h
WEAVE_HZ = (0.1, 2.0)
SHOWN = ('stanley.k_dist', 'stanley.k_soft', 'stanley.k_rate', 'stanley.k_steer',
         'stanley.max_ratio', 'stanley.fade_lo_mps', 'stanley.fade_hi_mps',
         'lookahead.gain', 'lookahead.min_m', 'lookahead.max_m', 'lookahead.curv_coef',
         'steer.rate_limit_deg_s', 'delay_comp_sec', 'mission.uniform_kmh')


def read_terms(path):
    """(t [s], terms [n x len(TERMS)]) from /wf_debug/terms, or None."""
    meta = rosbag2_py.Info().read_metadata(path, '')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=meta.storage_identifier),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/wf_debug/terms' not in types:
        return None
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/wf_debug/terms']))
    cls = get_message(types['/wf_debug/terms'])
    ts, rows = [], []
    while reader.has_next():
        _, raw, t = reader.read_next()
        data = deserialize_message(raw, cls).data
        if len(data) < len(TERMS):
            continue
        ts.append(t * 1e-9)
        rows.append(data[:len(TERMS)])
    if len(ts) < 2:
        return None
    return np.array(ts) - ts[0], np.array(rows)


def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flatten(v, prefix + k + '.'))
        else:
            out[prefix + k] = v
    return out


def read_params(path):
    f = os.path.join(path, 'wf_params.yaml')
    if not os.path.isfile(f):
        return {}
    with open(f) as fh:
        doc = yaml.safe_load(fh) or {}
    node = next(iter(doc.values()), {}) or {}
    return flatten(node.get('ros__parameters', {}))


def runs(mask, min_len):
    """Start/end indices of the stretches of True at least min_len long."""
    edges = np.flatnonzero(np.diff(np.concatenate(([0], mask.astype(int), [0]))))
    return [(a, b) for a, b in zip(edges[::2], edges[1::2]) if b - a >= min_len]


def weave_band(x, fs):
    """x band-passed to WEAVE_HZ (zero phase); NaN counts as 0."""
    hi = min(WEAVE_HZ[1], 0.45 * fs)
    sos = signal.butter(2, (WEAVE_HZ[0], hi), 'bandpass', fs=fs, output='sos')
    return signal.sosfiltfilt(sos, np.nan_to_num(x))


def weave(cte, segs, fs):
    """Dominant frequency [Hz] and amplitude [m] of cte over the given runs."""
    nfft = int(20 * fs)      # one frequency grid for every run
    f_all, p_all, w_all = None, 0.0, 0
    for a, b in segs:
        x = signal.detrend(np.nan_to_num(cte[a:b]))
        f_all, p = signal.welch(x, fs=fs, nperseg=min(len(x), nfft), nfft=nfft)
        p_all = p_all + p * (b - a)
        w_all += b - a
    if f_all is None:
        return float('nan'), float('nan')
    p_all = p_all / w_all
    m = (f_all >= WEAVE_HZ[0]) & (f_all <= WEAVE_HZ[1])
    if not m.any():
        return float('nan'), float('nan')
    k = np.flatnonzero(m)[np.argmax(p_all[m])]
    # Power within +-1 bin of the peak, as the amplitude of a sine.
    lo, hi = max(k - 1, 0), min(k + 2, len(f_all))
    amp = np.sqrt(2.0 * np.sum(p_all[lo:hi]) * (f_all[1] - f_all[0]))
    return f_all[k], amp


def fb_lag(cmd, fb, segs, fs):
    """Lag [s] of the measured wheel angle behind the command."""
    best, best_c = float('nan'), 0.0
    for k in range(0, int(1.0 * fs) + 1):
        c = 0.0
        for a, b in segs:
            u, y = cmd[a:b - k] if k else cmd[a:b], fb[a + k:b]
            ok = ~(np.isnan(u) | np.isnan(y))
            if ok.sum() > 10:
                c += np.dot(u[ok] - u[ok].mean(), y[ok] - y[ok].mean())
        if c > best_c:
            best, best_c = k / fs, c
    return best


def analyse(path):
    name = os.path.basename(os.path.normpath(path))
    got = read_terms(path)
    print(f'\n══ {name} ══')
    if got is None:
        print('   no /wf_debug/terms in this bag: record it with record_fit_bag.sh '
              'after rebuilding waypoint_follower.')
        return
    t, x = got
    col = {n: x[:, i] for i, n in enumerate(TERMS)}
    fs = 1.0 / np.median(np.diff(t))
    params = read_params(path)
    if params:
        print('   ' + '  '.join(f'{k.split(".", 1)[-1] if k.startswith("stanley.") else k}='
                                f'{params[k]}' for k in SHOWN if k in params))
    else:
        print('   (no wf_params.yaml: gains of this run unknown)')
    print(f'   {t[-1]:.0f} s, {fs:.0f} Hz, speed up to {np.nanmax(col["v_mps"]) * 3.6:.1f} km/h, '
          f'hunting detector active {100 * np.mean(col["osc_factor"] < 1.0):.1f} % of the time')

    ratio = np.nan_to_num(col['ratio'])
    parts = {'pp': (1.0 - ratio) * col['pp_deg'], 'head': ratio * col['heading_deg'],
             'cross': ratio * col['cross_deg'], 'rate': ratio * col['rate_deg'],
             'lag': ratio * col['lag_deg']}
    hf = {k: weave_band(v, fs) for k, v in parts.items()}
    cmd_hf = weave_band(col['cmd_deg'], fs)
    vk = col['v_mps'] * 3.6

    print(f'   {"km/h":>7s} {"sec":>5s} {"cte rms":>7s} {"p95":>5s} {"weave Hz":>8s} {"amp m":>6s} '
          f'{"cmd hf":>6s} | {"pp":>5s} {"head":>5s} {"cross":>5s} {"rate":>5s} {"lag":>5s} | '
          f'{"fb lag":>6s}  largest')
    for lo, hi in BANDS:
        m = (vk >= lo) & (vk < hi) & (col['v_mps'] > 0.3) & ~np.isnan(col['cte_m'])
        if m.sum() < 2 * fs:
            continue
        e = col['cte_m'][m]
        segs = runs(m, int(5 * fs))
        f_w, a_w = weave(col['cte_m'], segs, fs) if segs else (float('nan'), float('nan'))
        lag = fb_lag(col['cmd_deg'], col['fb_steer_deg'], segs, fs) if segs else float('nan')
        std = {k: np.std(v[m]) for k, v in hf.items()}
        top = max(std, key=std.get)
        print(f'   {lo:3d}-{hi:<3d} {m.sum() / fs:5.0f} {np.sqrt(np.mean(e ** 2)):7.3f} '
              f'{np.percentile(np.abs(e), 95):5.2f} {f_w:8.2f} {a_w:6.3f} {np.std(cmd_hf[m]):6.2f} | '
              f'{std["pp"]:5.2f} {std["head"]:5.2f} {std["cross"]:5.2f} {std["rate"]:5.2f} '
              f'{std["lag"]:5.2f} | {lag:6.2f}  {top}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+', help='rosbag2 directories')
    args = ap.parse_args()
    for b in args.bags:
        if not os.path.isdir(b):
            sys.exit(f'{b}: not a bag directory')
        analyse(b)


if __name__ == '__main__':
    main()
