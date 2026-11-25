#!/usr/bin/env python3
"""
Mapping metrics for every run under paper/data/mapping (or given run dirs).

Usage: python3 paper/experiments/metrics_mapping.py [--out <dir>] [--nav2-params <yaml>]
                                                    [run_dir ...]

Per run:
  t_explore     first frontier goal -> "Exploration complete" (sim s)
  t_return      "Exploration complete" -> "Back at start" (sim s)
  rtf           sim time / wall time over the run (Gazebo world stats)
  area_known    known cells (free + occupied) x resolution^2 in the final map (m^2)
  coverage      known cells whose centre lies inside the 10 x 8 m room / 80 m^2
  unknown_in_obstacles / unknown_outside
                unknown room cells whose centre lies inside / outside an obstacle
                footprint at the scan plane (m^2); together 80 m^2 x (1 - coverage)
  unknown_charger_corner
                part of unknown_outside in the charger corner (x > 3.6, y > 3.1 m),
                behind the back panel and beside the rails (m^2)
  err_mean/rms  distance from occupied cell centres to the nearest true surface
                at the scan plane (world SDF)
  width/length  wall-to-wall distances from the median occupied cell position
                in bands along each wall, against 10 m and 8 m
  progress_failures, goal_timeouts, goal_aborts, goals, recoveries
  min clearance of the ground-truth footprint to obstacles, the charger plate
  and the keepout zone (negative = overlap); the footprint is the one the run
  was recorded with (world_geometry.run_footprint)

Writes <out>/mapping_runs.csv, mapping_summary.json, mapping_curves.csv.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import re
import numpy as np
import world_geometry as wg


DATA = wg.WS / 'paper' / 'data'
CHARGER_CORNER = (3.6, 3.1)  # lower x, y bounds of the corner behind the charger panel and rails


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def events(run_dir):
    return [(float(r['sim_t']), r['node'], int(r['level']), r['msg'])
            for r in read_csv(run_dir / 'events.csv')]


def first(evs, node, text):
    for t, n, _, m in evs:
        if node in n and text in m:
            return t
    return None


def count(evs, node, text, t0=-math.inf, t1=math.inf):
    return sum(1 for t, n, _, m in evs if node in n and text in m and t0 <= t <= t1)


BEHAVIOUR_RE = re.compile(r'\[behavior_server\]: (Running (spin|backup|wait|drive_on_heading)'
                          r'|Turning [-0-9.]+ for spin)')


def behaviour_runs(launch_log):
    """Spin/back-up/wait runs; the behavior server logs them only to stdout."""
    text = Path(launch_log).read_text(errors='replace')
    return sum(1 for line in text.splitlines() if BEHAVIOUR_RE.search(line))


def load_maps(run_dir):
    z = np.load(run_dir / 'maps.npz')
    out = []
    for i, t in enumerate(z['t']):
        out.append((float(t), z[f'grid_{i:04d}'], float(z['resolution'][i]),
                    float(z['origin_x'][i]), float(z['origin_y'][i])))
    return out


def cell_centres(grid, res, ox, oy, mask):
    rows, cols = np.nonzero(mask)
    return ox + (cols + 0.5) * res, oy + (rows + 0.5) * res


def map_quality(grid, res, ox, oy, obstacles):
    known = grid >= 0
    occupied = grid > 50
    area_known = known.sum() * res * res
    xs, ys = cell_centres(grid, res, ox, oy, known)
    in_room = ((xs > wg.ROOM_X[0]) & (xs < wg.ROOM_X[1])
               & (ys > wg.ROOM_Y[0]) & (ys < wg.ROOM_Y[1]))
    coverage = in_room.sum() * res * res / wg.ROOM_AREA
    ux, uy = cell_centres(grid, res, ox, oy, grid < 0)
    keep = ((ux > wg.ROOM_X[0]) & (ux < wg.ROOM_X[1])
            & (uy > wg.ROOM_Y[0]) & (uy < wg.ROOM_Y[1]))
    ux, uy = ux[keep], uy[keep]
    inside = np.min([s.signed_distance(ux, uy) for s in obstacles], axis=0) < 0
    corner = ~inside & (ux > CHARGER_CORNER[0]) & (uy > CHARGER_CORNER[1])
    ox_, oy_ = cell_centres(grid, res, ox, oy, occupied)
    d = wg.nearest_surface_distance(ox_, oy_, obstacles)

    # Wall positions: mean occupied cell centre in a band along each wall
    # (cells within 0.15 m of the band median), away from corners and from
    # obstacles near the walls. Cell centres quantize this to the 0.05 m grid.
    def band(sel, coord):
        v = coord[sel]
        if not v.size:
            return math.nan
        v = v[np.abs(v - np.median(v)) < 0.15]
        return float(v.mean())

    west = band((ox_ < -4.5) & (np.abs(oy_) < 2.5), ox_)
    east = band((ox_ > 4.5) & (np.abs(oy_) < 2.5), ox_)
    south = band((oy_ < -3.5) & (np.abs(ox_) < 4.0), oy_)
    north = band((oy_ > 3.5) & (np.abs(ox_) < 3.0), oy_)
    return {
        'wall_west_off': west - wg.ROOM_X[0], 'wall_east_off': east - wg.ROOM_X[1],
        'wall_south_off': south - wg.ROOM_Y[0], 'wall_north_off': north - wg.ROOM_Y[1],
        'area_known': area_known,
        'area_free': (grid == 0).sum() * res * res,
        'coverage': coverage,
        'unknown_in_obstacles': inside.sum() * res * res,
        'unknown_outside': (~inside).sum() * res * res,
        'unknown_charger_corner': corner.sum() * res * res,
        'n_occupied': int(occupied.sum()),
        'err_mean': float(d.mean()),
        'err_rms': float(np.sqrt((d ** 2).mean())),
        'err_p95': float(np.percentile(d, 95)),
        'err_max': float(d.max()),
        'width': east - west,
        'length': north - south,
        'width_err': east - west - (wg.ROOM_X[1] - wg.ROOM_X[0]),
        'length_err': north - south - (wg.ROOM_Y[1] - wg.ROOM_Y[0]),
    }


def footprint_clearances(gt, obstacles, plate, keepout, fp, step=0.1):
    """Minimum ground-truth footprint clearances, sampled every `step` s.

    fp: footprint bounds (xmin, xmax, ymin, ymax) in the robot frame.
    """
    t = gt[:, 0]
    keep = np.concatenate([[True], np.diff(np.floor(t / step)) > 0])
    best = {'obstacle': (math.inf, ''), 'plate': (math.inf, ''), 'keepout': (math.inf, '')}
    for _, x, y, yaw in gt[keep][:, [0, 1, 2, 4]]:
        for s in obstacles:
            # Cheap reject: every footprint corner lies within 1.0 m of the robot
            # frame (0.39 m centred, 0.56 m with base_footprint on the drive axle).
            if abs(float(s.signed_distance(x, y))) > 1.0 and float(s.signed_distance(x, y)) > 0:
                continue
            c = wg.footprint_clearance(x, y, yaw, s, fp)
            if c < best['obstacle'][0]:
                best['obstacle'] = (c, s.name)
        for key, shape in (('plate', plate), ('keepout', keepout)):
            c = wg.footprint_clearance(x, y, yaw, shape, fp)
            if c < best[key][0]:
                best[key] = (c, shape.name)
    return best


def rtf(run_dir, t0, t1):
    rows = read_csv(run_dir / 'world_stats.csv')
    wall = np.array([float(r['wall_t']) for r in rows])
    sim = np.array([float(r['sim_t']) for r in rows])
    w0, w1 = np.interp([t0, t1], sim, wall)
    return (t1 - t0) / (w1 - w0)


def cm_interventions(run_dir):
    """Collision-monitor activations (transitions from idle to an action)."""
    n, prev = 0, 0
    for r in read_csv(run_dir / 'cm_state.csv'):
        a = int(r['action_type'])
        n += a != 0 and prev == 0
        prev = a
    return n


def run_metrics(run_dir, obstacles, plate, keepout, nav2_params=None):
    evs = events(run_dir)
    t_start = first(evs, 'frontier_explorer', 'Navigating to frontier')
    t_done = first(evs, 'frontier_explorer', 'Exploration complete')
    t_back = first(evs, 'frontier_explorer', 'Back at start')
    t_end = t_back if t_back is not None else t_done
    maps = load_maps(run_dir)
    final = [m for m in maps if t_end is None or m[0] <= t_end + 3.0][-1]
    q = map_quality(final[1], final[2], final[3], final[4], obstacles)

    gt = np.array([[float(v) for v in (r['sim_t'], r['x'], r['y'], r['z'], r['yaw'])]
                   for r in read_csv(run_dir / 'gt_pose.csv')])
    # Clearances cover the whole run: up to "Back at start", or to the end of
    # the recording when the robot never got back.
    t_clear_end = t_back if t_back is not None else gt[-1, 0]
    gt_run = gt[(gt[:, 0] >= t_start) & (gt[:, 0] <= t_clear_end)]
    clear = footprint_clearances(gt_run, obstacles, plate, keepout,
                                 wg.run_footprint(run_dir, nav2_params))
    gt_path = gt[(gt[:, 0] >= t_start) & (gt[:, 0] <= (t_end or gt[-1, 0]))]
    path_len = float(np.hypot(*np.diff(gt_path[::6, 1:3], axis=0).T).sum())

    fb = read_csv(run_dir / 'nav_feedback.csv')
    rec = {}
    for r in fb:
        rec[r['goal']] = max(rec.get(r['goal'], 0), int(r['number_of_recoveries']))

    row = {
        'run': run_dir.name,
        'completed': t_back is not None,
        't_explore': (t_done - t_start) if t_done is not None else math.nan,
        't_return': (t_back - t_done) if t_back is not None else math.nan,
        't_total': (t_back - t_start) if t_back is not None else math.nan,
        'rtf': rtf(run_dir, t_start, t_end if t_end else gt[-1, 0]),
        'path_length': path_len,
        'goals': count(evs, 'frontier_explorer', 'Navigating to frontier'),
        'explored_before_arrival': count(evs, 'frontier_explorer', 'explored before arrival'),
        'goal_timeouts': count(evs, 'frontier_explorer', 'Goal timed out'),
        'goal_aborts': count(evs, 'frontier_explorer', 'Nav2 aborted goal'),
        'goal_rejections': count(evs, 'frontier_explorer', 'Goal rejected'),
        'progress_failures': count(evs, 'controller_server', 'Failed to make progress'),
        'recoveries': sum(rec.values()),
        'behaviors_run': behaviour_runs(run_dir / 'launch.log'),
        'clear_obstacle': clear['obstacle'][0],
        'clear_obstacle_name': clear['obstacle'][1],
        'clear_plate': clear['plate'][0],
        'clear_keepout': clear['keepout'][0],
        'cm_interventions': cm_interventions(run_dir),
        **q,
    }
    curve = [(m[0] - t_start, (m[1] >= 0).sum() * m[2] ** 2) for m in maps
             if m[0] >= t_start - 5.0 and (t_end is None or m[0] <= t_end + 3.0)]
    # The very first map, published before exploration starts, anchors t = 0.
    pre = [m for m in maps if m[0] < t_start]
    if pre:
        curve.insert(0, (0.0, (pre[-1][1] >= 0).sum() * pre[-1][2] ** 2))
    return row, curve


def summarize(rows, keys):
    out = {}
    for k in keys:
        v = np.array([r[k] for r in rows], dtype=float)
        v = v[~np.isnan(v)]
        out[k] = {'n': int(v.size), 'mean': float(v.mean()), 'std': float(v.std(ddof=1))
                  if v.size > 1 else 0.0, 'min': float(v.min()), 'max': float(v.max())}
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default=str(DATA / 'results'))
    parser.add_argument('--nav2-params', default=None,
                        help='footprint source for runs without a nav2_params.yaml '
                             'snapshot (default: the repo file)')
    parser.add_argument('runs', nargs='*')
    args = parser.parse_args()
    runs = [Path(r) for r in args.runs] or sorted((DATA / 'mapping').glob('run_*'))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    obstacles, plate = wg.load_world()
    keepout = wg.keepout_box()

    rows, curves = [], []
    for run_dir in runs:
        row, curve = run_metrics(run_dir, obstacles, plate, keepout, args.nav2_params)
        rows.append(row)
        curves += [(run_dir.name, t, a) for t, a in curve]
        print(f"{row['run']}: total {row['t_total']:.1f} s (explore {row['t_explore']:.1f}, "
              f"return {row['t_return']:.1f}), area {row['area_known']:.2f} m2, coverage "
              f"{100 * row['coverage']:.1f} %, err {100 * row['err_mean']:.1f} cm, "
              f"progress failures {row['progress_failures']}, timeouts {row['goal_timeouts']}, "
              f"aborts {row['goal_aborts']}, rtf {row['rtf']:.3f}, min clear "
              f"{row['clear_obstacle']:.3f} ({row['clear_obstacle_name']}), keepout "
              f"{row['clear_keepout']:.3f}")

    with open(out / 'mapping_runs.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(out / 'mapping_curves.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run', 't_rel', 'area_known'])
        w.writerows([(r, f'{t:.3f}', f'{a:.4f}') for r, t, a in curves])
    numeric = [k for k, v in rows[0].items() if isinstance(v, (int, float)) and
               not isinstance(v, bool)]
    summary = {'runs': len(rows), 'completed': sum(r['completed'] for r in rows),
               'metrics': summarize(rows, numeric),
               'min_clearance_overall': min(r['clear_obstacle'] for r in rows),
               'min_keepout_clearance_overall': min(r['clear_keepout'] for r in rows),
               'min_plate_clearance_overall': min(r['clear_plate'] for r in rows)}
    (out / 'mapping_summary.json').write_text(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
