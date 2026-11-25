#!/usr/bin/env python3
"""
Fig. 4: map growth in one run and explored area over all runs.

Usage: python3 fig_mapping.py [run_dir]   (default: the median-duration mapping run)

(a)-(c): /map of the run at two times and at the end, with the ground-truth
path driven so far; (c) also overlays the true obstacle outlines at the scan
plane (world SDF), the charger plate and the keepout zone. (d): explored area
(known cells x resolution^2) against time since the first frontier goal,
mean +/- 1 std over all runs (paper/data/results/mapping_curves.csv), with the
80 m^2 room area for reference.
"""

import csv
import json
import math
from pathlib import Path
import sys

from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon, Rectangle
import matplotlib.pyplot as plt
import numpy as np

import style

sys.path.insert(0, str(style.WS / 'paper/experiments'))
import world_geometry as wg  # noqa: E402

SNAP_TIMES = (15.0, 45.0)


def median_run():
    rows = [r for r in csv.DictReader(open(style.DATA / 'results/mapping_runs.csv'))
            if r['completed'] == 'True']
    rows.sort(key=lambda r: float(r['t_total']))
    return rows[len(rows) // 2]


def first_goal_time(run):
    for r in csv.DictReader(open(run / 'events.csv')):
        if 'frontier_explorer' in r['node'] and 'Navigating to frontier' in r['msg']:
            return float(r['sim_t'])
    raise RuntimeError('no frontier goal in ' + str(run))


def draw_map(ax, grid, res, ox, oy):
    h, w = grid.shape
    ax.imshow(style.grid_rgb(grid), origin='lower', extent=[ox, ox + w * res, oy, oy + h * res],
              interpolation='none')


def outline(ax, shape, **kw):
    if isinstance(shape, wg.Circle):
        ax.add_patch(plt.Circle((shape.cx, shape.cy), shape.r, fill=False, **kw))
    else:
        ax.add_patch(Polygon(shape.corners(), closed=True, fill=False, **kw))


def frame(ax):
    ax.set_xlim(-5.35, 5.35)
    ax.set_ylim(-4.35, 4.35)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_linewidth(0.4)
        sp.set_color(style.MUTED)


def scale_bar(ax, length=2.0):
    ax.plot([-5.1, -5.1 + length], [-4.08, -4.08], color=style.INK, lw=1.2,
            solid_capstyle='butt', zorder=7)
    ax.text(-5.1 + length / 2, -4.0, f'{length:g} m', ha='center', va='bottom', fontsize=8,
            color=style.INK, zorder=7)


def main():
    if len(sys.argv) > 1:
        run = Path(sys.argv[1])
        row = None
    else:
        row = median_run()
        run = style.DATA / 'mapping' / row['run']
    t0 = first_goal_time(run)
    z = np.load(run / 'maps.npz')
    gt = np.array([[float(r[k]) for k in ('sim_t', 'x', 'y')]
                   for r in csv.DictReader(open(run / 'gt_pose.csv'))])
    obstacles, plate = wg.load_world()
    keep = wg.keepout_box()

    t_maps = z['t'] - t0
    end_t = (float(row['t_total']) if row else t_maps[-1])
    idx = [int(np.argmin(np.abs(t_maps - t))) for t in SNAP_TIMES]
    idx.append(int(np.flatnonzero(t_maps <= end_t + 3.0)[-1]))

    style.apply()
    fig = plt.figure(figsize=(style.TEXT_W, 1.62))
    gs = fig.add_gridspec(1, 3, wspace=0.05, left=0.003, right=0.68, top=0.90, bottom=0.25)
    gs_plot = fig.add_gridspec(1, 1, left=0.80, right=0.995, top=0.90, bottom=0.25)
    meta = {'run': run.name, 'panels': []}
    for k, i in enumerate(idx):
        ax = fig.add_subplot(gs[0, k])
        grid = z[f'grid_{i:04d}']
        res = float(z['resolution'][i])
        draw_map(ax, grid, res, float(z['origin_x'][i]), float(z['origin_y'][i]))
        t_rel = float(t_maps[i])
        path = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t0 + t_rel)]
        ax.plot(path[::6, 1], path[::6, 2], color=style.BLUE, lw=0.6, zorder=4)
        ax.plot(path[-1, 1], path[-1, 2], marker='o', ms=2.5, color=style.BLUE, zorder=5)
        area = float((grid >= 0).sum() * res * res)
        if k == 2:
            for s in obstacles:
                if not s.name.startswith('wall'):
                    outline(ax, s, ec=style.ORANGE, lw=0.5, zorder=5)
            ax.add_patch(Rectangle((wg.ROOM_X[0], wg.ROOM_Y[0]), wg.ROOM_X[1] - wg.ROOM_X[0],
                                   wg.ROOM_Y[1] - wg.ROOM_Y[0], fill=False, ec=style.ORANGE,
                                   lw=0.5, zorder=5))
            outline(ax, plate, ec=style.ORANGE, lw=0.5, ls=(0, (2, 1.2)), zorder=5)
            ax.add_patch(Rectangle((keep.cx - keep.sx / 2, keep.cy - keep.sy / 2), keep.sx,
                                   keep.sy, fill=False, hatch='//////', lw=0.5, ec=style.RED,
                                   zorder=5))
        frame(ax)
        if k == 0:
            scale_bar(ax)
        end = ' (end)' if k == 2 else ''
        style.panel_label(ax, f'({"abc"[k]}) t = {t_rel:.0f} s{end}, {area:.1f} m$^2$', y=1.02)
        meta['panels'].append({'t_rel': t_rel, 'area': area})

    ax = fig.add_subplot(gs_plot[0, 0])
    curves = {}
    for r in csv.DictReader(open(style.DATA / 'results/mapping_curves.csv')):
        curves.setdefault(r['run'], []).append((float(r['t_rel']), float(r['area_known'])))
    t_end = max(max(t for t, _ in c) for c in curves.values())
    grid_t = np.arange(0.0, math.ceil(t_end) + 1.0, 0.5)
    areas = []
    for c in curves.values():
        c = np.array(sorted(c))
        idx_c = np.searchsorted(c[:, 0], grid_t, side='right') - 1
        areas.append(np.where(idx_c >= 0, c[np.clip(idx_c, 0, None), 1], c[0, 1]))
    areas = np.array(areas)
    mean, std = areas.mean(axis=0), areas.std(axis=0, ddof=1) if len(areas) > 1 else 0 * grid_t
    for a in areas:
        ax.plot(grid_t, a, color=style.MUTED, lw=0.35, zorder=2)
    ax.fill_between(grid_t, mean - std, mean + std, color=style.BLUE, alpha=0.22, lw=0,
                    zorder=3)
    ax.plot(grid_t, mean, color=style.BLUE, lw=1.1, zorder=4)
    ax.axhline(wg.ROOM_AREA, color=style.INK_2, lw=0.7, ls=(0, (4, 2)), zorder=1)
    ax.set_xlim(0, grid_t[-1])
    ax.set_ylim(0, 85)
    ax.set_xlabel('Time since first goal (s)')
    ax.set_ylabel('Explored area (m$^2$)')
    ax.spines[['top', 'right']].set_visible(False)
    ax.grid(axis='y', lw=0.3, color='#e6e5e1')
    ax.set_axisbelow(True)
    style.panel_label(ax, '(d)', x=-0.25, y=1.02)
    ax.legend(handles=[Line2D([], [], color=style.BLUE, lw=1.1, label='Mean'),
                       Patch(fc=style.BLUE, alpha=0.22, label=r'$\pm$1 std'),
                       Line2D([], [], color=style.MUTED, lw=0.5, label=f'{len(areas)} runs'),
                       Line2D([], [], color=style.INK_2, lw=0.7, ls=(0, (4, 2)),
                              label='Room, 80 m$^2$')],
              loc='lower right', fontsize=8, handlelength=1.5, borderaxespad=0.2,
              labelspacing=0.25)

    fig.legend(handles=[Patch(fc=style.FREE, ec=style.MUTED, lw=0.4, label='Free'),
                        Patch(fc=style.UNKNOWN, label='Unknown'),
                        Patch(fc=style.OCCUPIED, label='Occupied'),
                        Line2D([], [], color=style.BLUE, lw=0.8, label='Robot path'),
                        Line2D([], [], color=style.ORANGE, lw=0.6, label='True obstacles'),
                        Patch(fc='white', ec=style.RED, hatch='//////', lw=0.5,
                              label='Keepout zone')],
               loc='lower left', bbox_to_anchor=(0.0, 0.0), ncol=6, fontsize=8,
               handlelength=1.3, columnspacing=1.1, handletextpad=0.4, borderaxespad=0.1)
    fig.savefig(style.FIGURES / 'fig_mapping.pdf')
    meta['area_final_mean'] = float(mean[-1])
    (style.DATA / 'results' / 'fig_mapping_meta.json').write_text(json.dumps(meta, indent=2))
    print('wrote', style.FIGURES / 'fig_mapping.pdf', 'from', run, meta)


if __name__ == '__main__':
    main()
