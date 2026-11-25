#!/usr/bin/env python3
"""
Metrics and statistics for the rotation-centre study (see ../README.md).

Usage (from the workspace root; run extract_bag.py on every run first):
  python3 research/rotation_center/analysis/compare.py

Conditions:
  A0  paper/data (2026-10-10, the course-paper code), navigation and mapping only
  A   research/rotation_center/data/A
  B   research/rotation_center/data/B

Writes to research/rotation_center/results/:
  <C>_nav_goals.csv, <C>_nav_sessions.csv, <C>_mapping_runs.csv, <C>_spins.csv
  spin_curves.csv     per-sample spin data for the figures
  comparison.json     summaries, tests and the pre-registered decisions
  summary_table.md    the main numbers side by side

Course-paper metric code is reused from paper/experiments (metrics_navigation,
metrics_mapping, world_geometry); body clearances use each run's own footprint.
"""

import csv
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy import stats as sps
import yaml

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / 'paper' / 'experiments'))
import metrics_mapping as mm  # noqa: E402
import metrics_navigation as mn  # noqa: E402
import world_geometry as wg  # noqa: E402

STUDY = WS / 'research' / 'rotation_center'
RESULTS = STUDY / 'results'
CONDITIONS = {'A0': WS / 'paper' / 'data', 'A': STUDY / 'data' / 'A', 'B': STUDY / 'data' / 'B'}
TOL_POS, TOL_YAW = 0.20, 0.15
BOOT_N, BOOT_SEED = 10000, 20261116
RATE_WARNING = 'Control loop missed its desired rate'


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    if not rows:
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def robot_points(urdf_text):
    """(axle_x, centre_x): drive-axle midpoint and chassis centre, x in base_footprint."""
    root = ET.fromstring(urdf_text)

    def x_of(el):
        o = el.find('origin')
        return float(o.get('xyz').split()[0]) if o is not None else 0.0

    base = x_of(root.find("joint[@name='base_footprint_joint']"))
    axle = base + x_of(root.find("joint[@name='left_wheel_joint']"))
    centre = base + x_of(root.find("link[@name='base_link']/visual"))
    return axle, centre


def a0_files():
    """nav2_params.yaml and properties.xacro of the course-paper code, saved in results/.

    A0 runs have no per-run snapshots.
    """
    params = RESULTS / 'A0_nav2_params.yaml'
    props = ET.parse(RESULTS / 'A0_properties.xacro').getroot()
    wheel_x = next(float(p.get('value')) for p in props
                   if p.tag.endswith('property') and p.get('name') == 'wheel_x_offset')
    return params, (wheel_x, 0.0)


def run_geometry(run_dir, a0_points):
    urdf = run_dir / 'robot_description.urdf'
    return robot_points(urdf.read_text()) if urdf.exists() else a0_points


def counted(cond_dir, kind):
    """Counted run directories of one kind, from runs.csv (in run order)."""
    rows = read_csv(cond_dir / 'runs.csv')
    return [cond_dir / r['run'] for r in rows
            if r['counted'] == 'yes' and r['run'].startswith(f'{kind}/')]


def lifecycle_audit(run_dir):
    """(managers that never reported 'Managed nodes are active', lost service replies).

    A lost DDS service reply can leave a lifecycle manager waiting forever without failing
    the run, e.g. the costmap-filter manager, which leaves the keepout filter off.
    """
    path = run_dir / 'events.csv'
    rows = read_csv(path if path.exists() else run_dir / 'events_live.csv')
    managers = {r['node'] for r in rows if r['node'].startswith('lifecycle_manager')}
    active = {r['node'] for r in rows if r['node'].startswith('lifecycle_manager')
              and 'Managed nodes are active' in r['msg']}
    lost = [r['msg'].split(' (timeout)')[0].replace('failed to send response to ', '')
            for r in rows if 'failed to send response' in r['msg']]
    return sorted(managers - active), lost


def audit(cond, cond_dir):
    """Every counted run must have all lifecycle managers active (A0 is only flagged)."""
    runs = ([p for kind in ('mapping', 'navigation') for p in sorted(
        (cond_dir / kind).glob('*_*')) if not p.name.startswith('infra')] if cond == 'A0'
        else [d for kind in ('spin', 'mapping', 'navigation') for d in counted(cond_dir, kind)])
    flagged = {}
    for d in runs:
        inactive, lost = lifecycle_audit(d)
        if inactive or lost:
            flagged[str(d.relative_to(WS))] = {'inactive_managers': inactive,
                                               'lost_replies': lost}
        if inactive and cond != 'A0':
            raise SystemExit(f'{d}: {inactive} never became active; this is an '
                             f'infrastructure failure and must not be counted')
    return flagged


def mission_window(sdir):
    """(first goal accepted, last goal ended) in sim s, as metrics_navigation defines it."""
    goals = mn.goals_from_status(sdir / 'nav_status.csv')
    gt_end = float(read_csv(sdir / 'gt_pose.csv')[-1]['sim_t'])
    return goals[0][1]['accept'], max((g['end'] or gt_end) for _, g in goals)


def rtf_between(run_dir, t0, t1):
    rows = read_csv(run_dir / 'world_stats.csv')
    wall = np.array([float(r['wall_t']) for r in rows])
    sim = np.array([float(r['sim_t']) for r in rows])
    w0, w1 = np.interp([t0, t1], sim, wall)
    return (t1 - t0) / (w1 - w0)


def rate_warnings(run_dir):
    return (run_dir / 'launch.log').read_text(errors='replace').count(RATE_WARNING)


def navigation(cond, cond_dir, a0_params, a0_points):
    sessions = (sorted((cond_dir / 'navigation').glob('session_*')) if cond == 'A0'
                else counted(cond_dir, 'navigation'))
    waypoints = mn.load_waypoints()
    obstacles, plate = wg.load_world()
    keepout = wg.keepout_box()
    goal_rows, sess_rows = [], []
    for sdir in sessions:
        snap = sdir / 'waypoints.yaml'
        if snap.exists():
            flat = yaml.safe_load(snap.read_text())['waypoint_runner']['ros__parameters'][
                'waypoints']
            if [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)] != waypoints:
                raise SystemExit(f'{sdir}: waypoints differ from the repo file')
        rows, s = mn.session_metrics(sdir, waypoints, obstacles, plate, keepout,
                                     a0_params if cond == 'A0' else None)
        axle_x, centre_x = run_geometry(sdir, a0_points)
        gt = mn.GroundTruth(sdir / 'gt_pose.csv')
        goals = mn.goals_from_status(sdir / 'nav_status.csv')
        for r, (_, g) in zip(rows, goals):
            wx, wy, _ = waypoints[r['waypoint'] - 1]
            x, y, yaw = gt.at(g['end'] if g['end'] is not None else gt.t[-1])
            c, si = math.cos(yaw), math.sin(yaw)
            r['condition'] = cond
            r['within_tol'] = bool(r['success'] and r['pos_err'] <= TOL_POS
                                   and r['yaw_err'] <= TOL_YAW)
            r['pos_err_axle'] = math.hypot(x + c * axle_x - wx, y + si * axle_x - wy)
            r['pos_err_centre'] = math.hypot(x + c * centre_x - wx, y + si * centre_x - wy)
            goal_rows.append(r)
        t0, t1 = mission_window(sdir)
        evs = mm.events(sdir)
        sess_rows.append({
            'condition': cond, 'session': sdir.name, 'goals': s['goals'],
            'succeeded': s['succeeded'],
            'within_tol': sum(r['within_tol'] for r in rows),
            'amcl_pos_rmse': s['amcl_pos_rmse'], 'amcl_pos_max': s['amcl_pos_max'],
            'amcl_yaw_rmse': s['amcl_yaw_rmse'],
            'recoveries': sum(r['recoveries'] for r in rows),
            'behaviours_run': s['behaviours_run'],
            'cm_interventions': s['cm_interventions'],
            'progress_failures': mm.count(evs, 'controller_server', 'Failed to make progress',
                                          t0, t1),
            'rate_warnings': rate_warnings(sdir),
            'rtf': rtf_between(sdir, t0, t1),
            'clear_obstacle': s['clear_obstacle'], 'clear_obstacle_name': s['clear_obstacle_name'],
            'clear_plate': s['clear_plate'], 'clear_keepout': s['clear_keepout'],
            'mission_time': s['mission_time'],
            'localizer_started': any('global_localizer' in n for _, n, _, _ in evs),
            'localizer_converged': s['localizer']['converged'],
        })
    return goal_rows, sess_rows


def mapping(cond, cond_dir, a0_params):
    runs = (sorted((cond_dir / 'mapping').glob('run_*')) if cond == 'A0'
            else counted(cond_dir, 'mapping'))
    obstacles, plate = wg.load_world()
    keepout = wg.keepout_box()
    rows = []
    for run_dir in runs:
        row, _ = mm.run_metrics(run_dir, obstacles, plate, keepout,
                                a0_params if cond == 'A0' else None)
        rows.append({'condition': cond, **row, 'rate_warnings': rate_warnings(run_dir)})
    return rows


def series(path):
    a = np.array([[float(r[k]) for k in ('sim_t', 'x', 'y', 'yaw')] for r in read_csv(path)])
    return a[:, 0], a[:, 1], a[:, 2], np.unwrap(a[:, 3])


def relative(x0, y0, th0, x, y):
    """Positions in the robot frame at t0."""
    c, s = math.cos(th0), math.sin(th0)
    dx, dy = x - x0, y - y0
    return c * dx + s * dy, -s * dx + c * dy


def spins(cond, cond_dir):
    rows, curves = [], []
    for sdir in counted(cond_dir, 'spin'):
        axle_x, _ = robot_points((sdir / 'robot_description.urdf').read_text())
        offset = abs(axle_x)
        evs = [(float(r['sim_t']), r['msg']) for r in read_csv(sdir / 'events.csv')
               if r['node'].endswith('spin_test')]
        gt_t, gt_x, gt_y, gt_th = series(sdir / 'gt_pose.csv')
        od_t, od_x, od_y, od_th = series(sdir / 'odom.csv')
        am_t, am_x, am_y, am_th = series(sdir / 'amcl_pose.csv')
        for k in (1, 2):
            t0 = next((t for t, m in evs if m.startswith(f'spin {k} start')), None)
            done = next(((t, m) for t, m in evs if m.startswith(f'spin {k} done')), None)
            if t0 is None or done is None:
                continue
            t1, msg = done
            sel = (gt_t >= t0) & (gt_t <= t1)
            t = gt_t[sel]

            def at(ts, vs, when):
                return np.interp(when, ts, vs)

            g0 = (at(gt_t, gt_x, t0), at(gt_t, gt_y, t0), at(gt_t, gt_th, t0))
            o0 = (at(od_t, od_x, t0), at(od_t, od_y, t0), at(od_t, od_th, t0))
            gx, gy = relative(*g0, gt_x[sel], gt_y[sel])
            ox, oy = relative(*o0, at(od_t, od_x, t), at(od_t, od_y, t))
            dth = gt_th[sel] - g0[2]
            disp = np.hypot(gx, gy)
            axx = gx + np.cos(dth) * axle_x - axle_x
            axy = gy + np.sin(dth) * axle_x
            odom_err = np.hypot(ox - gx, oy - gy)
            predicted = 2.0 * offset * np.abs(np.sin(dth / 2.0))
            am_sel = (am_t >= t0) & (am_t <= t1)
            a0 = (at(am_t, am_x, t0), at(am_t, am_y, t0), at(am_t, am_th, t0))
            ax, ay = relative(*a0, am_x[am_sel], am_y[am_sel])
            gax, gay = relative(*g0, at(gt_t, gt_x, am_t[am_sel]), at(gt_t, gt_y, am_t[am_sel]))
            amcl_err = np.hypot(ax - gax, ay - gay)
            rows.append({
                'condition': cond, 'session': sdir.name, 'spin': k,
                'result': msg.split(':', 1)[1].strip(),
                'commanded': float(next(m for _, m in evs if m.startswith(f'spin {k} start'))
                                   .split(':')[1].split()[0]),
                'rotation_gt': float(dth[-1]),
                'duration': t1 - t0,
                'base_offset_to_axle': offset,
                'peak_displacement': float(disp.max()),
                'axle_drift_max': float(np.hypot(axx, axy).max()),
                'axle_drift_end': float(np.hypot(axx[-1], axy[-1])),
                'odom_err_max': float(odom_err.max()),
                'odom_err_rms_vs_pred': float(np.sqrt(np.mean((odom_err - predicted) ** 2))),
                'amcl_updates': int(am_sel.sum()),
                'amcl_err_max': float(amcl_err.max()) if amcl_err.size else math.nan,
            })
            curves += [{'condition': cond, 'session': sdir.name, 'spin': k,
                        't': f'{ti - t0:.3f}', 'dtheta': f'{d:.5f}', 'gt_x': f'{a:.5f}',
                        'gt_y': f'{b:.5f}', 'odom_x': f'{c:.5f}', 'odom_y': f'{e:.5f}',
                        'odom_err': f'{f:.5f}', 'predicted': f'{p:.5f}'}
                       for ti, d, a, b, c, e, f, p in zip(t, dth, gx, gy, ox, oy, odom_err,
                                                          predicted)]
    return rows, curves


def describe(v):
    v = np.asarray([x for x in v if x is not None and not math.isnan(x)], dtype=float)
    if v.size == 0:
        return None
    return {'n': int(v.size), 'mean': float(v.mean()),
            'std': float(v.std(ddof=1)) if v.size > 1 else 0.0,
            'median': float(np.median(v)), 'min': float(v.min()), 'max': float(v.max())}


def mwu(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if a.size == 0 or b.size == 0:
        return None
    res = sps.mannwhitneyu(a, b, alternative='two-sided')
    return {'U': float(res.statistic), 'p': float(res.pvalue), 'n_a': int(a.size),
            'n_b': int(b.size), 'median_a': float(np.median(a)), 'median_b': float(np.median(b))}


def cluster_bootstrap(goals_a, goals_b, stat):
    """95% CI of stat(B) - stat(A), resampling whole sessions within each condition."""
    rng = np.random.default_rng(BOOT_SEED)

    def by_session(goals):
        out = {}
        for g in goals:
            out.setdefault(g['session'], []).append(g)
        return list(out.values())

    sa, sb = by_session(goals_a), by_session(goals_b)
    diffs = []
    for _ in range(BOOT_N):
        ra = [g for i in rng.integers(len(sa), size=len(sa)) for g in sa[i]]
        rb = [g for i in rng.integers(len(sb), size=len(sb)) for g in sb[i]]
        diffs.append(stat(rb) - stat(ra))
    lo, hi = np.percentile(diffs, [2.5, 97.5])
    return {'estimate': stat(goals_b) - stat(goals_a), 'ci95': [float(lo), float(hi)],
            'resamples': BOOT_N, 'seed': BOOT_SEED}


def share_within(goals):
    return sum(g['within_tol'] for g in goals) / len(goals)


def mean_pos_err(goals):
    ok = [g['pos_err'] for g in goals if g['success']]
    return float(np.mean(ok)) if ok else math.nan


def compare_nav(goals, sessions, a, b):
    ga = [g for g in goals if g['condition'] == a]
    gb = [g for g in goals if g['condition'] == b]
    sa = [s for s in sessions if s['condition'] == a]
    sb = [s for s in sessions if s['condition'] == b]
    ka, kb = sum(g['within_tol'] for g in ga), sum(g['within_tol'] for g in gb)
    odds, p = sps.fisher_exact([[kb, len(gb) - kb], [ka, len(ga) - ka]])
    ok_a = [g['pos_err'] for g in ga if g['success']]
    ok_b = [g['pos_err'] for g in gb if g['success']]
    yaw_a = [g['yaw_err'] for g in ga if g['success']]
    yaw_b = [g['yaw_err'] for g in gb if g['success']]
    return {
        'within_tolerance': {a: [ka, len(ga)], b: [kb, len(gb)],
                             'fisher_odds_ratio_b_vs_a': float(odds), 'fisher_p': float(p),
                             'share_diff_bootstrap': cluster_bootstrap(ga, gb, share_within)},
        'pos_err': {'mwu': mwu(ok_a, ok_b),
                    'mean_diff_bootstrap': cluster_bootstrap(ga, gb, mean_pos_err)},
        'yaw_err': {'mwu': mwu(yaw_a, yaw_b)},
        'amcl_pos_rmse': {'mwu': mwu([s['amcl_pos_rmse'] for s in sa],
                                     [s['amcl_pos_rmse'] for s in sb])},
        'progress_failures': {a: sum(s['progress_failures'] for s in sa),
                              b: sum(s['progress_failures'] for s in sb),
                              'mwu_per_session': mwu([s['progress_failures'] for s in sa],
                                                     [s['progress_failures'] for s in sb])},
        'recoveries': {a: sum(s['recoveries'] for s in sa), b: sum(s['recoveries'] for s in sb),
                       'mwu_per_session': mwu([s['recoveries'] for s in sa],
                                              [s['recoveries'] for s in sb])},
        'cm_interventions': {a: sum(s['cm_interventions'] for s in sa),
                             b: sum(s['cm_interventions'] for s in sb),
                             'mwu_per_session': mwu([s['cm_interventions'] for s in sa],
                                                    [s['cm_interventions'] for s in sb])},
        'min_clearance_obstacle': {a: min(s['clear_obstacle'] for s in sa),
                                   b: min(s['clear_obstacle'] for s in sb)},
    }


MAP_KEYS = ['t_explore', 't_return', 't_total', 'area_known', 'coverage', 'err_mean',
            'progress_failures', 'goal_timeouts', 'goal_aborts', 'recoveries',
            'cm_interventions', 'clear_obstacle', 'rtf', 'rate_warnings']


def compare_map(rows, a, b):
    ra = [r for r in rows if r['condition'] == a]
    rb = [r for r in rows if r['condition'] == b]
    out = {'completed': {a: [sum(r['completed'] for r in ra), len(ra)],
                         b: [sum(r['completed'] for r in rb), len(rb)]}}
    for k in MAP_KEYS:
        va = [r[k] for r in ra if not (isinstance(r[k], float) and math.isnan(r[k]))]
        vb = [r[k] for r in rb if not (isinstance(r[k], float) and math.isnan(r[k]))]
        out[k] = mwu(va, vb)
    return out


def summarize(cond, goals, sessions, maps, spin_rows):
    g = [x for x in goals if x['condition'] == cond]
    s = [x for x in sessions if x['condition'] == cond]
    m = [x for x in maps if x['condition'] == cond]
    p = [x for x in spin_rows if x['condition'] == cond]
    ok = [x for x in g if x['success']]
    out = {}
    if g:
        out['navigation'] = {
            'sessions': len(s), 'goals': len(g), 'succeeded': len(ok),
            'within_tol': sum(x['within_tol'] for x in g),
            'pos_within': sum(x['pos_err'] <= TOL_POS for x in ok),
            'yaw_within': sum(x['yaw_err'] <= TOL_YAW for x in ok),
            'pos_err': describe([x['pos_err'] for x in ok]),
            'yaw_err': describe([x['yaw_err'] for x in ok]),
            'pos_err_axle': describe([x['pos_err_axle'] for x in ok]),
            'pos_err_centre': describe([x['pos_err_centre'] for x in ok]),
            'time': describe([x['time'] for x in ok]),
            'path_length': describe([x['path_length'] for x in ok]),
            'per_waypoint': {k: {'within_tol': sum(x['within_tol'] for x in g
                                                   if x['waypoint'] == k),
                                 'n': sum(1 for x in g if x['waypoint'] == k),
                                 'pos_err': describe([x['pos_err'] for x in ok
                                                      if x['waypoint'] == k])}
                             for k in sorted({x['waypoint'] for x in g})},
            'amcl_pos_rmse': describe([x['amcl_pos_rmse'] for x in s]),
            'progress_failures': sum(x['progress_failures'] for x in s),
            'recoveries': sum(x['recoveries'] for x in s),
            'cm_interventions': sum(x['cm_interventions'] for x in s),
            'rate_warnings': sum(x['rate_warnings'] for x in s),
            'rtf': describe([x['rtf'] for x in s]),
            'min_clearance_obstacle': min(x['clear_obstacle'] for x in s),
            'min_clearance_keepout': min(x['clear_keepout'] for x in s),
            'localizer_converged': [sum(x['localizer_converged'] for x in s),
                                    sum(x['localizer_started'] for x in s)],
            'localizer_not_started': sum(not x['localizer_started'] for x in s),
        }
    if m:
        out['mapping'] = {'runs': len(m), 'completed': sum(x['completed'] for x in m),
                          **{k: describe([x[k] for x in m]) for k in MAP_KEYS},
                          'min_clearance_obstacle': min(x['clear_obstacle'] for x in m)}
    if p:
        out['spin'] = {'spins': len(p),
                       'succeeded': sum(x['result'] == 'SUCCEEDED' for x in p),
                       'abs_rotation_gt': describe([abs(x['rotation_gt']) for x in p]),
                       **{k: describe([x[k] for x in p])
                          for k in ('duration', 'peak_displacement', 'axle_drift_max',
                                    'axle_drift_end', 'odom_err_max', 'odom_err_rms_vs_pred',
                                    'amcl_err_max')}}
    return out


def h1_decision(summary):
    """Pre-registered H1 thresholds (README.md)."""
    a, b = summary.get('A', {}).get('spin'), summary.get('B', {}).get('spin')
    if not a or not b:
        return None
    a_peak = a['peak_displacement']['median']
    return {
        'A_peak_displacement_median_in_0.36_0.44': 0.36 <= a_peak <= 0.44,
        'A_odom_err_rms_vs_pred_median_le_0.03': a['odom_err_rms_vs_pred']['median'] <= 0.03,
        'B_peak_displacement_median_le_0.05': b['peak_displacement']['median'] <= 0.05,
    }


def _ms(d, nd=3):
    """'mean ± std (median)' of a describe() dict."""
    if not d:
        return '–'
    return f'{d["mean"]:.{nd}f} ± {d["std"]:.{nd}f} ({d["median"]:.{nd}f})'


def _frac(k, n):
    return f'{k}/{n} ({100 * k / n:.0f} %)' if n else '–'


def _p(x):
    return '–' if x is None else (f'{x:.3g}' if x >= 0.001 else f'{x:.1e}')


def summary_table(comparison):
    """Markdown table of the main numbers, A0 | A | B, plus the B vs A tests."""
    summ = comparison['summary']
    conds = [c for c in CONDITIONS if c in summ]

    def row(label, fn):
        cells = []
        for c in conds:
            try:
                cells.append(fn(summ[c]))
            except (KeyError, TypeError):
                cells.append('–')
        return f'| {label} | ' + ' | '.join(cells) + ' |'

    head = '| Metric | ' + ' | '.join(conds) + ' |\n|---|' + '---|' * len(conds)
    nav = lambda s: s['navigation']  # noqa: E731
    out = ['# Rotation-centre study: summary', '',
           'Generated by `analysis/compare.py` from `results/comparison.json`. Values are '
           'mean ± std (median) unless stated. Errors are against Gazebo ground truth, for '
           'goals Nav2 reported as reached.', '', '## Navigation', '', head,
           row('Sessions / goals', lambda s: f'{nav(s)["sessions"]} / {nav(s)["goals"]}'),
           row('Reached (Nav2)', lambda s: _frac(nav(s)['succeeded'], nav(s)['goals'])),
           row('**Within 0.20 m and 0.15 rad**',
               lambda s: '**' + _frac(nav(s)['within_tol'], nav(s)['goals']) + '**'),
           row('Position within 0.20 m', lambda s: _frac(nav(s)['pos_within'],
                                                         nav(s)['succeeded'])),
           row('Yaw within 0.15 rad', lambda s: _frac(nav(s)['yaw_within'],
                                                      nav(s)['succeeded'])),
           row('Final position error (m)', lambda s: _ms(nav(s)['pos_err'])),
           row('Final yaw error (rad)', lambda s: _ms(nav(s)['yaw_err'])),
           row('Error of the axle midpoint (m)', lambda s: _ms(nav(s)['pos_err_axle'])),
           row('Error of the chassis centre (m)', lambda s: _ms(nav(s)['pos_err_centre'])),
           row('Time per goal (s)', lambda s: _ms(nav(s)['time'], 1)),
           row('Path length per goal (m)', lambda s: _ms(nav(s)['path_length'], 2)),
           row('AMCL position RMSE per session (m)', lambda s: _ms(nav(s)['amcl_pos_rmse'])),
           row('Progress-checker failures (total)', lambda s: str(nav(s)['progress_failures'])),
           row('Recoveries (total)', lambda s: str(nav(s)['recoveries'])),
           row('Collision-monitor interventions (total)',
               lambda s: str(nav(s)['cm_interventions'])),
           row('MPPI missed-rate warnings (total)', lambda s: str(nav(s)['rate_warnings'])),
           row('Min. body clearance to obstacles (m)',
               lambda s: f'{nav(s)["min_clearance_obstacle"]:.3f}'),
           row('Min. body clearance to keepout (m)',
               lambda s: f'{nav(s)["min_clearance_keepout"]:.3f}'),
           row('Real-time factor', lambda s: _ms(nav(s)['rtf'], 3)),
           row('Global localizer converged / trials started',
               lambda s: f'{nav(s)["localizer_converged"][0]} / '
                         f'{nav(s)["localizer_converged"][1]}'),
           '', '## Mapping', '', head]
    mp = lambda s: s['mapping']  # noqa: E731
    out += [row('Runs completed (back at start)',
                lambda s: _frac(mp(s)['completed'], mp(s)['runs'])),
            row('Exploration time (s)', lambda s: _ms(mp(s)['t_explore'], 1)),
            row('Total time (s)', lambda s: _ms(mp(s)['t_total'], 1)),
            row('Known area (m²)', lambda s: _ms(mp(s)['area_known'], 2)),
            row('Coverage of the 80 m² room', lambda s: _ms(mp(s)['coverage'], 4)),
            row('Map error, mean (m)', lambda s: _ms(mp(s)['err_mean'], 4)),
            row('Progress-checker failures per run', lambda s: _ms(mp(s)['progress_failures'], 2)),
            row('Recoveries per run', lambda s: _ms(mp(s)['recoveries'], 2)),
            row('Collision-monitor interventions per run',
                lambda s: _ms(mp(s)['cm_interventions'], 2)),
            row('Min. body clearance to obstacles (m)',
                lambda s: f'{mp(s)["min_clearance_obstacle"]:.3f}'),
            row('Real-time factor', lambda s: _ms(mp(s)['rtf'], 3)),
            '', '## Spin test', '', head]
    sp = lambda s: s['spin']  # noqa: E731
    out += [row('Spins succeeded', lambda s: _frac(sp(s)['succeeded'], sp(s)['spins'])),
            row('True rotation, absolute (rad)', lambda s: _ms(sp(s)['abs_rotation_gt'], 4)),
            row('Peak true displacement (m)', lambda s: _ms(sp(s)['peak_displacement'], 4)),
            row('Max. slip of the axle midpoint (m)', lambda s: _ms(sp(s)['axle_drift_max'], 4)),
            row('Axle-midpoint slip at spin end (m)', lambda s: _ms(sp(s)['axle_drift_end'], 4)),
            row('Max. odometry error (m)', lambda s: _ms(sp(s)['odom_err_max'], 4)),
            row('RMS deviation from predicted error (m)',
                lambda s: _ms(sp(s)['odom_err_rms_vs_pred'], 4)),
            row('Max. AMCL error (m)', lambda s: _ms(sp(s)['amcl_err_max'], 4))]

    t = comparison['tests'].get('B_vs_A')
    if t:
        n, m = t['navigation'], t['mapping']
        wt, pe = n['within_tolerance'], n['pos_err']

        def ci(b):
            return f'{b["estimate"]:+.3f} [{b["ci95"][0]:+.3f}, {b["ci95"][1]:+.3f}]'

        out += ['', '## B vs A: tests', '',
                '| Test | Effect (B − A) [95 % cluster-bootstrap CI] | p |', '|---|---|---|',
                f'| H2 within tolerance, Fisher exact | share {ci(wt["share_diff_bootstrap"])}; '
                f'odds ratio {wt["fisher_odds_ratio_b_vs_a"]:.2f} | {_p(wt["fisher_p"])} |',
                f'| H2 final position error, Mann–Whitney U | mean '
                f'{ci(pe["mean_diff_bootstrap"])} m; medians {pe["mwu"]["median_a"]:.3f} → '
                f'{pe["mwu"]["median_b"]:.3f} m | {_p(pe["mwu"]["p"])} |',
                f'| Final yaw error, Mann–Whitney U | medians '
                f'{n["yaw_err"]["mwu"]["median_a"]:.3f} → {n["yaw_err"]["mwu"]["median_b"]:.3f}'
                f' rad | {_p(n["yaw_err"]["mwu"]["p"])} |',
                f'| H3 AMCL RMSE per session, Mann–Whitney U | medians '
                f'{n["amcl_pos_rmse"]["mwu"]["median_a"]:.3f} → '
                f'{n["amcl_pos_rmse"]["mwu"]["median_b"]:.3f} m | '
                f'{_p(n["amcl_pos_rmse"]["mwu"]["p"])} |']
        for k, label in (('progress_failures', 'H4 progress failures'),
                         ('recoveries', 'H4 recoveries'),
                         ('cm_interventions', 'H4 collision-monitor interventions')):
            mw = n[k]['mwu_per_session']
            out.append(f'| {label} per session, Mann–Whitney U | totals {n[k]["A"]} → '
                       f'{n[k]["B"]} | {_p(mw["p"]) if mw else "–"} |')
        for k in ('t_explore', 't_total', 'coverage', 'err_mean'):
            mw = m[k]
            out.append(f'| Mapping {k}, Mann–Whitney U | medians {mw["median_a"]:.4g} → '
                       f'{mw["median_b"]:.4g} | {_p(mw["p"])} |')
    h1 = comparison.get('h1_decision')
    if h1:
        out += ['', '## H1 thresholds (pre-registered)', '']
        out += [f'- `{k}`: {"met" if v else "not met"}' for k, v in h1.items()]
    return '\n'.join(out) + '\n'


def main():
    RESULTS.mkdir(parents=True, exist_ok=True)
    a0_params, a0_points = a0_files()
    goals, sessions, maps, spin_rows, curves = [], [], [], [], []
    audits = {}
    for cond, cdir in CONDITIONS.items():
        if cond != 'A0' and not (cdir / 'runs.csv').exists():
            print(f'{cond}: no runs yet, skipped')
            continue
        audits[cond] = audit(cond, cdir)
        g, s = navigation(cond, cdir, a0_params, a0_points)
        m = mapping(cond, cdir, a0_params)
        p, c = spins(cond, cdir) if cond != 'A0' else ([], [])
        for name, rows in (('nav_goals', g), ('nav_sessions', s), ('mapping_runs', m),
                           ('spins', p)):
            write_csv(RESULTS / f'{cond}_{name}.csv', rows)
        goals += g
        sessions += s
        maps += m
        spin_rows += p
        curves += c
        print(f'{cond}: {len(s)} navigation sessions ({len(g)} goals), {len(m)} mapping runs, '
              f'{len(p)} spins')
    write_csv(RESULTS / 'spin_curves.csv', curves)

    present = sorted({x['condition'] for x in goals}, key=list(CONDITIONS).index)
    summary = {c: summarize(c, goals, sessions, maps, spin_rows) for c in present}
    comparison = {'summary': summary, 'lifecycle_audit': audits, 'tests': {}}
    for a, b in (('A0', 'A'), ('A', 'B'), ('A0', 'B')):
        if a in present and b in present:
            comparison['tests'][f'{b}_vs_{a}'] = {'navigation': compare_nav(goals, sessions, a, b),
                                                  'mapping': compare_map(maps, a, b)}
    comparison['h1_decision'] = h1_decision(summary)
    (RESULTS / 'comparison.json').write_text(json.dumps(comparison, indent=2, default=str) + '\n')
    (RESULTS / 'summary_table.md').write_text(summary_table(comparison))
    print(f'wrote {RESULTS}')


if __name__ == '__main__':
    main()
