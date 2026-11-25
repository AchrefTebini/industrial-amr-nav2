#!/usr/bin/env python3
"""
Navigation metrics for every session under paper/data/navigation (or given dirs).

Usage: python3 paper/experiments/metrics_navigation.py [--out <dir>] [--nav2-params <yaml>]
                                                       [session_dir ...]

Goals come from the NavigateToPose status in the bag, ordered by acceptance
time; waypoint_runner sends the waypoints of waypoints.yaml in order, so goal
i is waypoint (i mod 5) + 1 of loop (i div 5) + 1. Errors are measured
against the Gazebo ground truth (gt_pose.csv), not against AMCL.

Writes <out>/nav_goals.csv and nav_summary.json.
"""


import argparse
import csv
import json
import math
from pathlib import Path
import re
import numpy as np
import yaml
import world_geometry as wg


DATA = wg.WS / 'paper' / 'data'
WAYPOINTS = wg.WS / 'src/industrial_amr_exploration/config/waypoints.yaml'
SUCCEEDED, CANCELED, ABORTED = 4, 5, 6


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_waypoints():
    flat = yaml.safe_load(WAYPOINTS.read_text())['waypoint_runner']['ros__parameters']['waypoints']
    return [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]


class GroundTruth:
    def __init__(self, path):
        a = np.array([[float(r[k]) for k in ('sim_t', 'x', 'y', 'yaw')] for r in read_csv(path)])
        self.t, self.x, self.y = a[:, 0], a[:, 1], a[:, 2]
        self.yaw_unwrapped = np.unwrap(a[:, 3])

    def at(self, t):
        return (float(np.interp(t, self.t, self.x)), float(np.interp(t, self.t, self.y)),
                wrap(float(np.interp(t, self.t, self.yaw_unwrapped))))

    def path_length(self, t0, t1, step=0.1):
        ts = np.arange(t0, t1, step)
        if ts.size < 2:
            return 0.0
        return float(np.hypot(np.diff(np.interp(ts, self.t, self.x)),
                              np.diff(np.interp(ts, self.t, self.y))).sum())

    def samples(self, t0, t1, step=0.1):
        ts = np.arange(t0, t1, step)
        return [self.at(t) for t in ts]


BEHAVIOUR_RE = re.compile(r'\[behavior_server-\d+\] \[INFO\] \[(\d+\.\d+)\] \[behavior_server\]: '
                          r'Running (spin|backup|wait|drive_on_heading)$')


def behaviour_runs(sdir, t0, t1):
    """Spin/back-up/wait runs inside [t0, t1] (sim s), from launch.log (stdout only)."""
    clock = read_csv(sdir / 'clock.csv')
    wall = np.array([float(r['wall_t']) for r in clock])
    sim = np.array([float(r['sim_t']) for r in clock])
    n = 0

    for line in (sdir / 'launch.log').read_text(errors='replace').splitlines():
        m = BEHAVIOUR_RE.search(line)
        if m and t0 <= float(np.interp(float(m.group(1)), wall, sim)) <= t1:
            n += 1
    return n


def goals_from_status(path):
    goals = {}
    for r in read_csv(path):
        g = goals.setdefault(r['goal'], {'accept': float(r['accept_sim_t']), 'end': None,
                                         'status': None})
        s = int(r['status'])

        if s in (SUCCEEDED, CANCELED, ABORTED) and g['end'] is None:
            g['end'], g['status'] = float(r['sim_t']), s
    return sorted(goals.items(), key=lambda kv: kv[1]['accept'])


def session_metrics(sdir, waypoints, obstacles, plate, keepout, nav2_params=None):
    gt = GroundTruth(sdir / 'gt_pose.csv')
    evs = [(float(r['sim_t']), r['node'], r['msg']) for r in read_csv(sdir / 'events.csv')]
    rec = {}

    for r in read_csv(sdir / 'nav_feedback.csv'):
        rec[r['goal']] = max(rec.get(r['goal'], 0), int(r['number_of_recoveries']))
    goals = goals_from_status(sdir / 'nav_status.csv')
    rows = []
    half = wg.run_footprint(sdir, nav2_params)

    for i, (gid, g) in enumerate(goals):
        wx, wy, wyaw = waypoints[i % len(waypoints)]
        end = g['end'] if g['end'] is not None else gt.t[-1]
        x, y, yaw = gt.at(end)
        rows.append({
            'session': sdir.name, 'loop': i // len(waypoints) + 1,
            'waypoint': i % len(waypoints) + 1, 'goal': gid,
            'status': {SUCCEEDED: 'succeeded', ABORTED: 'aborted',
                       CANCELED: 'canceled'}.get(g['status'], 'none'),
            'success': g['status'] == SUCCEEDED,
            'time': end - g['accept'],
            'path_length': gt.path_length(g['accept'], end),
            'pos_err': math.hypot(x - wx, y - wy),
            'yaw_err': abs(wrap(yaw - wyaw)),
            'recoveries': rec.get(gid, 0),
        })

    t_first = goals[0][1]['accept'] if goals else gt.t[0]
    t_last = max((g['end'] or gt.t[-1]) for _, g in goals) if goals else gt.t[-1]

    # AMCL error over the waypoint mission (the global-localizer trial comes after it).
    amcl = [(float(r['sim_t']), float(r['x']), float(r['y']), float(r['yaw']))
            for r in read_csv(sdir / 'amcl_pose.csv')]

    mission = [a for a in amcl if t_first <= a[0] <= t_last]
    pos_err, yaw_err, dx, dy = [], [], [], []
    for t, ax, ay, ayaw in mission:
        x, y, yaw = gt.at(t)
        pos_err.append(math.hypot(ax - x, ay - y))
        yaw_err.append(abs(wrap(ayaw - yaw)))
        dx.append(ax - x)
        dy.append(ay - y)

    # Collision monitor interventions: transitions into a non-idle action.
    cm = [(float(r['sim_t']), int(r['action_type'])) for r in read_csv(sdir / 'cm_state.csv')]
    interventions, prev = 0, 0
    for t, a in cm:
        if t_first <= t <= t_last and a != 0 and prev == 0:
            interventions += 1
        prev = a

    # Footprint clearances over the mission.
    best = {'obstacle': (math.inf, ''), 'plate': math.inf, 'keepout': math.inf}
    for x, y, yaw in gt.samples(t_first, t_last):
        for s in obstacles:
            if float(s.signed_distance(x, y)) > 1.0:
                continue
            c = wg.footprint_clearance(x, y, yaw, s, half)
            if c < best['obstacle'][0]:
                best['obstacle'] = (c, s.name)
        best['plate'] = min(best['plate'], wg.footprint_clearance(x, y, yaw, plate, half))
        best['keepout'] = min(best['keepout'], wg.footprint_clearance(x, y, yaw, keepout, half))

    # Global-localizer trial after the mission.
    loc = [(t, m) for t, n, m in evs if 'global_localizer' in n]
    t_scatter = next((t for t, m in loc if 'Scattering particles' in m), None)
    t_conv = next((t for t, m in loc if 'converged' in m.lower()), None)
    converged = any('Localisation converged' in m for _, m in loc)
    spins = sum(1 for _, m in loc if 'Spinning to converge' in m)
    stddevs = [float(m.split(':')[1].split()[0]) for _, m in loc if 'Position std-dev' in m]
    loc_err = None

    if t_conv is not None:
        after = [a for a in amcl if a[0] <= t_conv + 1.0]
        if after:
            t, ax, ay, ayaw = after[-1]
            x, y, yaw = gt.at(t)
            loc_err = {'pos': math.hypot(ax - x, ay - y), 'yaw': abs(wrap(ayaw - yaw))}

    behaviours = behaviour_runs(sdir, t_first, t_last)

    session = {
        'session': sdir.name, 'goals': len(rows),
        'succeeded': sum(r['success'] for r in rows),
        'amcl_n': len(mission),
        'amcl_pos_rmse': float(np.sqrt(np.mean(np.square(pos_err)))) if pos_err else math.nan,
        'amcl_pos_max': float(np.max(pos_err)) if pos_err else math.nan,
        'amcl_yaw_rmse': float(np.sqrt(np.mean(np.square(yaw_err)))) if yaw_err else math.nan,
        'amcl_yaw_max': float(np.max(yaw_err)) if yaw_err else math.nan,
        'amcl_bias_x': float(np.mean(dx)) if dx else math.nan,
        'amcl_bias_y': float(np.mean(dy)) if dy else math.nan,
        'cm_interventions': interventions,
        'behaviours_run': behaviours,
        'clear_obstacle': best['obstacle'][0], 'clear_obstacle_name': best['obstacle'][1],
        'clear_plate': best['plate'], 'clear_keepout': best['keepout'],
        'mission_time': t_last - t_first,
        'localizer': {'converged': converged, 'spins': spins, 'stddevs': stddevs,
                      'time': (t_conv - t_scatter) if t_scatter and t_conv else None,
                      'final_err': loc_err},
    }
    return rows, session


def stats(v):
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return None

    return {'n': int(v.size), 'mean': float(v.mean()),
            'std': float(v.std(ddof=1)) if v.size > 1 else 0.0,
            'min': float(v.min()), 'max': float(v.max())}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default=str(DATA / 'results'))
    parser.add_argument('--nav2-params', default=None,
                        help='footprint source for sessions without a nav2_params.yaml '
                             'snapshot (default: the repo file)')
    parser.add_argument('sessions', nargs='*')
    args = parser.parse_args()
    sessions = ([Path(s) for s in args.sessions]
                or sorted((DATA / 'navigation').glob('session_*')))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    waypoints = load_waypoints()
    obstacles, plate = wg.load_world()
    keepout = wg.keepout_box()

    rows, sess = [], []
    for sdir in sessions:
        r, s = session_metrics(sdir, waypoints, obstacles, plate, keepout, args.nav2_params)
        rows += r
        sess.append(s)
        print(json.dumps({k: v for k, v in s.items() if k != 'localizer'}, default=str))
        print('  localizer:', s['localizer'])

    with open(out / 'nav_goals.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    per_wp = {}
    for k in range(1, len(waypoints) + 1):
        rk = [r for r in rows if r['waypoint'] == k]
        ok = [r for r in rk if r['success']]
        per_wp[k] = {'n': len(rk), 'succeeded': len(ok),
                     'time': stats([r['time'] for r in ok]),
                     'path_length': stats([r['path_length'] for r in ok]),
                     'pos_err': stats([r['pos_err'] for r in ok]),
                     'yaw_err': stats([r['yaw_err'] for r in ok])}

    ok = [r for r in rows if r['success']]
    amcl_sq = [(s['amcl_pos_rmse'] ** 2, s['amcl_n']) for s in sess if s['amcl_n']]
    yaw_sq = [(s['amcl_yaw_rmse'] ** 2, s['amcl_n']) for s in sess if s['amcl_n']]
    n_amcl = sum(n for _, n in amcl_sq)
    summary = {
        'sessions': len(sess), 'goals': len(rows), 'succeeded': len(ok),
        'per_waypoint': per_wp,
        'overall': {'time': stats([r['time'] for r in ok]),
                    'path_length': stats([r['path_length'] for r in ok]),
                    'pos_err': stats([r['pos_err'] for r in ok]),
                    'yaw_err': stats([r['yaw_err'] for r in ok]),
                    'within_tolerance': sum(1 for r in ok if r['pos_err'] <= 0.20
                                            and r['yaw_err'] <= 0.15)},
        'amcl_pos_rmse': math.sqrt(sum(e * n for e, n in amcl_sq) / n_amcl) if n_amcl else None,
        'amcl_pos_max': max(s['amcl_pos_max'] for s in sess),
        'amcl_yaw_rmse': math.sqrt(sum(e * n for e, n in yaw_sq) / n_amcl) if n_amcl else None,
        'amcl_yaw_max': max(s['amcl_yaw_max'] for s in sess),
        'recoveries': sum(r['recoveries'] for r in rows),
        'cm_interventions': sum(s['cm_interventions'] for s in sess),
        'min_clearance_obstacle': min(s['clear_obstacle'] for s in sess),
        'min_clearance_plate': min(s['clear_plate'] for s in sess),
        'min_clearance_keepout': min(s['clear_keepout'] for s in sess),
        'session_details': sess,
    }
    (out / 'nav_summary.json').write_text(json.dumps(summary, indent=2, default=str))


if __name__ == '__main__':
    main()
