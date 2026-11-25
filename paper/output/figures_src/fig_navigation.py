#!/usr/bin/env python3
"""
Fig. 5: waypoint missions on the committed map, from Gazebo ground truth.

Usage: python3 fig_navigation.py [session_dir ...]   (default: paper/data/navigation/session_*)

Background: src/industrial_amr_navigation/maps/industrial_room.yaml (the map
navigation localizes against). Paths: ground-truth base_footprint positions
over each session's waypoint mission (first goal accepted -> last goal done),
one colour and line style per session. Also drawn: the waypoints of
waypoints.yaml, the keepout zone from the mask and the charger plate (SDF).
"""

import csv
from pathlib import Path
import sys

from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
import matplotlib.pyplot as plt
import numpy as np
import yaml

import style

sys.path.insert(0, str(style.WS / 'paper/experiments'))
import world_geometry as wg  # noqa: E402

MAP_YAML = style.WS / 'src/industrial_amr_navigation/maps/industrial_room.yaml'
WAYPOINTS = style.WS / 'src/industrial_amr_exploration/config/waypoints.yaml'


def mission_window(sdir):
    accept, ends = [], []
    for r in csv.DictReader(open(sdir / 'nav_status.csv')):
        accept.append(float(r['accept_sim_t']))
        if int(r['status']) in (4, 5, 6):
            ends.append(float(r['sim_t']))
    return min(accept), max(ends)


def main():
    sessions = [Path(s) for s in sys.argv[1:]] or sorted(
        (style.DATA / 'navigation').glob('session_*'))
    grid, res, ox, oy = wg.load_map_yaml(MAP_YAML)
    flat = yaml.safe_load(WAYPOINTS.read_text())['waypoint_runner']['ros__parameters'][
        'waypoints']
    wps = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
    _, plate = wg.load_world()
    keep = wg.keepout_box()

    style.apply()
    fig = plt.figure(figsize=(style.COLUMN_W, 2.1))
    ax = fig.add_axes([0.002, 0.004, 0.74, 0.992])
    h, w = grid.shape
    ax.imshow(style.grid_rgb(grid), origin='lower', extent=[ox, ox + w * res, oy, oy + h * res],
              interpolation='none')
    ax.add_patch(Rectangle((keep.cx - keep.sx / 2, keep.cy - keep.sy / 2), keep.sx, keep.sy,
                           fill=False, hatch='//////', lw=0.5, ec=style.RED, zorder=3))
    ax.add_patch(Rectangle((plate.cx - plate.sx / 2, plate.cy - plate.sy / 2), plate.sx,
                           plate.sy, fill=False, lw=0.6, ls=(0, (2, 1.2)), ec=style.INK,
                           zorder=4))

    handles = []
    for k, sdir in enumerate(sessions):
        t0, t1 = mission_window(sdir)
        gt = np.array([[float(r[c]) for c in ('sim_t', 'x', 'y')]
                       for r in csv.DictReader(open(sdir / 'gt_pose.csv'))])
        gt = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
        colour, ls = style.SERIES[k % len(style.SERIES)]
        ax.plot(gt[::6, 1], gt[::6, 2], color=colour, ls=ls, lw=0.8, zorder=5)
        handles.append(Line2D([], [], color=colour, ls=ls, lw=0.9, label=f'Session {k + 1}'))

    ax.plot(0, 0, marker='s', ms=4, mfc='white', mec=style.INK, mew=0.7, zorder=6)
    ax.text(0.12, 0.12, 'start', fontsize=8, color=style.INK, ha='left', va='bottom', zorder=6)
    for i, (x, y, _) in enumerate(wps, start=1):
        ax.plot(x, y, marker='o', ms=8.5, mfc='white', mec=style.INK, mew=0.7, zorder=7)
        ax.text(x, y, str(i), fontsize=8, color=style.INK, ha='center', va='center', zorder=8)

    ax.plot([-4.8, -2.8], [-3.75, -3.75], color=style.INK, lw=1.2, solid_capstyle='butt',
            zorder=7)
    ax.text(-3.8, -3.68, '2 m', ha='center', va='bottom', fontsize=8, color=style.INK)
    ax.set_xlim(-5.25, 5.25)
    ax.set_ylim(-4.25, 4.25)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_linewidth(0.4)
        sp.set_color(style.MUTED)

    handles += [
        Line2D([], [], ls='', marker='o', ms=6, mfc='white', mec=style.INK, mew=0.7,
               label='Waypoint'),
        Patch(fc='white', ec=style.RED, hatch='//////', lw=0.5, label='Keepout zone'),
        Patch(fc='none', ec=style.INK, ls=(0, (2, 1.2)), lw=0.6, label='Charger plate'),
    ]
    fig.legend(handles=handles, loc='center left', bbox_to_anchor=(0.75, 0.5), ncol=1,
               fontsize=8, handlelength=1.6, handletextpad=0.4, labelspacing=0.6,
               borderaxespad=0.0)
    fig.savefig(style.FIGURES / 'fig_navigation.pdf', bbox_inches=style.full_figure(fig))
    print('wrote', style.FIGURES / 'fig_navigation.pdf', [s.name for s in sessions])


if __name__ == '__main__':
    main()
