#!/usr/bin/env python3
"""
Fig. 3: frontier detection and goal selection on real /map snapshots.

Usage: python3 fig_frontiers.py [run_dir]   (default: the median-duration mapping run)

Runs the package's own frontier code (frontier_detection.find_frontiers and
Frontier.goal) with the parameters of frontier_explorer.yaml on two maps of a
logged run: (a) the start-up map, whose only frontier is a ring around the
robot, and (b) the first map with several clusters. The selected goal uses
the explorer's score, size_weight * length - distance_weight * distance
(frontier_explorer.py, select_frontier). The robot pose is the SLAM estimate at
the map's time (est_pose.csv).
"""

import csv
import json
import math
from pathlib import Path
import sys

from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import numpy as np
import yaml

import style

sys.path.insert(0, str(style.WS / 'src/industrial_amr_exploration'))
from industrial_amr_exploration.frontier_detection import find_frontiers, GridInfo  # noqa: E402

PARAMS = yaml.safe_load((style.WS / 'src/industrial_amr_exploration/config/'
                         'frontier_explorer.yaml').read_text())['frontier_explorer'][
                             'ros__parameters']


def median_run():
    rows = list(csv.DictReader(open(style.DATA / 'results/mapping_runs.csv')))
    rows = [r for r in rows if r['completed'] == 'True']
    rows.sort(key=lambda r: float(r['t_total']))
    return style.DATA / 'mapping' / rows[len(rows) // 2]['run']


def select(grid, info, robot):
    res = info.resolution
    frontiers = find_frontiers(grid, info,
                               min_size_cells=max(1, round(PARAMS['min_frontier_size'] / res)),
                               occupied_clearance_cells=PARAMS['obstacle_clearance'] / res)
    out, best, best_score = [], None, -math.inf
    for f in frontiers:
        goal = f.goal(robot[0], robot[1], PARAMS['min_goal_distance'])
        score = None
        if goal is not None:
            dist = math.hypot(goal[0] - robot[0], goal[1] - robot[1])
            score = PARAMS['size_weight'] * f.size * res - PARAMS['distance_weight'] * dist
            if score > best_score:
                best, best_score = len(out), score
        out.append((f, goal, score))
    return out, best


def draw(ax, grid, info, robot, frontiers, best, half):
    h, w = grid.shape
    ext = [info.origin_x, info.origin_x + w * info.resolution,
           info.origin_y, info.origin_y + h * info.resolution]
    ax.imshow(style.grid_rgb(grid), origin='lower', extent=ext,
              interpolation='none')
    for i, (f, goal, _) in enumerate(frontiers):
        chosen = i == best
        cell_pt = 120.0 * info.resolution / (2 * half)
        ax.scatter(f.xs, f.ys, s=max(cell_pt ** 2, 1.0), marker='s', lw=0,
                   color=style.ORANGE if chosen else style.BLUE, zorder=3)
        ax.plot(f.xs.mean(), f.ys.mean(), marker='x', ms=3.5, mew=0.8, color=style.INK, zorder=4)
        if goal is not None and not chosen:
            ax.plot(*goal, marker='o', ms=2.8, mfc='white', mec=style.BLUE, mew=0.8, zorder=5)
    if best is not None:
        ax.plot(*frontiers[best][1], marker='*', ms=7, mfc=style.ORANGE, mec=style.INK, mew=0.5,
                zorder=6)
    ax.add_patch(Circle(robot[:2], PARAMS['min_goal_distance'], fill=False, ls=(0, (3, 2)),
                        lw=0.6, ec=style.INK_2, zorder=4))
    x, y, yaw = robot
    ax.plot(x, y, marker=(3, 0, math.degrees(yaw) - 90), ms=5, color=style.INK, zorder=6)
    ax.set_facecolor(style.UNKNOWN)
    ax.set_xlim(x - half, x + half)
    ax.set_ylim(y - half, y + half)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_linewidth(0.4)
        sp.set_color(style.MUTED)


def scale_bar(ax, length=1.0):
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    bx, by = x0 + 0.06 * (x1 - x0), y0 + 0.06 * (y1 - y0)
    ax.plot([bx, bx + length], [by, by], color=style.INK, lw=1.2, solid_capstyle='butt',
            zorder=7)
    ax.text(bx + length / 2, by + 0.025 * (y1 - y0), f'{length:g} m', ha='center', va='bottom',
            fontsize=8, color=style.INK, zorder=7,
            bbox=dict(fc='white', ec='none', pad=0.5, alpha=0.85))


def main():
    run = Path(sys.argv[1]) if len(sys.argv) > 1 else median_run()
    z = np.load(run / 'maps.npz')
    est = np.array([[float(r[k]) for k in ('sim_t', 'x', 'y', 'yaw')]
                    for r in csv.DictReader(open(run / 'est_pose.csv'))])

    def snapshot(i):
        t = float(z['t'][i])
        info = GridInfo(float(z['resolution'][i]), float(z['origin_x'][i]),
                        float(z['origin_y'][i]))
        robot = (float(np.interp(t, est[:, 0], est[:, 1])),
                 float(np.interp(t, est[:, 0], est[:, 2])),
                 float(np.interp(t, est[:, 0], np.unwrap(est[:, 3]))))
        grid = z[f'grid_{i:04d}']
        return t, grid, info, robot, *select(grid, info, robot)

    snaps = [snapshot(0)]
    area0 = float((z['grid_0000'] >= 0).sum())
    for i in range(1, len(z['t'])):
        if (z[f'grid_{i:04d}'] >= 0).sum() < 3 * area0:
            continue
        s = snapshot(i)
        if len(s[4]) >= 3:
            snaps.append(s)
            break

    style.apply()
    fig, axes = plt.subplots(1, 2, figsize=(style.COLUMN_W, 2.05),
                             gridspec_kw=dict(wspace=0.06, left=0.01, right=0.99, top=0.93,
                                              bottom=0.27))
    meta = {}

    def fit_half(robot, frontiers, minimum):
        pts = [(f.xs.mean(), f.ys.mean()) for f, _, _ in frontiers]
        pts += [g for _, g, _ in frontiers if g is not None]
        far = max(max(abs(x - robot[0]), abs(y - robot[1])) for x, y in pts)
        return max(minimum, math.ceil((far + 0.3) * 10) / 10)

    halves = [fit_half(s[3], s[4], 1.6) for s in snaps]
    for ax, (t, grid, info, robot, frontiers, best), label, half in zip(
            axes, snaps, ('(a)', '(b)'), halves):
        draw(ax, grid, info, robot, frontiers, best, half)
        scale_bar(ax, 1.0)
        style.panel_label(ax, f'{label} t = {t:.1f} s, {len(frontiers)} cluster'
                          f'{"s" if len(frontiers) != 1 else ""}', y=1.015)
        meta[label] = {'t': t, 'clusters': len(frontiers),
                       'known_area': float((grid >= 0).sum() * info.resolution ** 2),
                       'robot': robot,
                       'centroid_to_robot': [math.hypot(f.xs.mean() - robot[0],
                                                        f.ys.mean() - robot[1])
                                             for f, _, _ in frontiers],
                       'selected_goal': frontiers[best][1] if best is not None else None,
                       'view_half_width': half,
                       'clusters_detail': [
                           {'cells': int(f.size), 'length_m': float(f.size * info.resolution),
                            'centroid': [float(f.xs.mean()), float(f.ys.mean())],
                            'goal': g, 'score': sc, 'selected': k == best}
                           for k, (f, g, sc) in enumerate(frontiers)]}

    handles = [
        Line2D([], [], ls='', marker='s', ms=3, color=style.BLUE, label='Frontier cells'),
        Line2D([], [], ls='', marker='s', ms=3, color=style.ORANGE, label='Selected cluster'),
        Line2D([], [], ls='', marker='x', ms=3.5, mew=0.8, color=style.INK, label='Centroid'),
        Line2D([], [], ls='', marker='o', ms=2.8, mfc='white', mec=style.BLUE, mew=0.8,
               label='Cluster goal'),
        Line2D([], [], ls='', marker='*', ms=7, mfc=style.ORANGE, mec=style.INK, mew=0.5,
               label='Selected goal'),
        Line2D([], [], ls=(0, (3, 2)), lw=0.6, color=style.INK_2,
               label=f"{PARAMS['min_goal_distance']:g} m from robot"),
    ]
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False, fontsize=8,
               handlelength=1.4, columnspacing=1.0, handletextpad=0.4, borderaxespad=0.1)
    fig.savefig(style.FIGURES / 'fig_frontiers.pdf', bbox_inches=style.full_figure(fig))
    (style.DATA / 'results').mkdir(exist_ok=True)
    (style.DATA / 'results' / 'fig_frontiers_meta.json').write_text(
        json.dumps({'run': run.name, **meta}, indent=2, default=float))
    print('wrote', style.FIGURES / 'fig_frontiers.pdf', 'from', run)
    print(json.dumps(meta, indent=1, default=float)[:1500])


if __name__ == '__main__':
    main()
