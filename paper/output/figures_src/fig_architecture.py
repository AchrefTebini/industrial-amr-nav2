#!/usr/bin/env python3
"""
Fig. 2: software architecture, drawn with explicit positions.

Content from the launch files (industrial_amr_bringup/launch,
industrial_amr_navigation/launch/nav2.launch.py, nav2_bringup navigation/localization
launch), nav2_params.yaml and the exploration nodes: the two modes share the Nav2
navigation servers; the explorer and the waypoint runner send NavigateToPose goals,
the global localizer re-initializes AMCL, reads amcl_pose and requests Spin; the
planner returns the path to the navigator, which hands it to the controller (default
BT: ComputePathToPose, then FollowPath); controller and recoveries publish cmd_vel_nav
to the velocity smoother, then the collision monitor, then Gazebo. Main nodes only (no
lifecycle managers, robot_state_publisher, smoother/route/docking/waypoint-follower
servers). Line style: solid = both modes, dashed = mapping only, dotted = navigation
only.
"""

from matplotlib.patches import FancyBboxPatch, Polygon
import matplotlib.pyplot as plt
import numpy as np

import style

style.apply()
W, H = style.TEXT_W, 2.25
BOTH, MAPPING, NAVIGATION = '-', (0, (4, 2.5)), (0, (1, 1.6))
EDGE = style.INK_2

fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis('off')


def box(x, y, w, h, text, ls=BOTH, lw=0.6, weight='normal'):
    """Rounded node centred at (x, y); returns its bounds for arrows."""
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle='round,pad=0,rounding_size=0.05',
                                fc='white', ec=EDGE, lw=lw, ls=ls))
    ax.text(x, y, text, ha='center', va='center', fontsize=8, color=style.INK,
            linespacing=1.1, fontweight=weight)
    return {'x': x, 'y': y, 'w': w, 'h': h}


def group(x0, y0, x1, y1, title):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0,
                                boxstyle='round,pad=0,rounding_size=0.07',
                                fc='#f6f6f4', ec=style.MUTED, lw=0.5))
    ax.text(x0 + 0.07, y1 - 0.05, title, ha='left', va='top', fontsize=8, color=style.INK_2,
            style='italic')


def side(b, where, frac=0.5):
    """Point on a node's side: 'l', 'r', 't', 'b'; frac along that side."""
    x, y, w, h = b['x'], b['y'], b['w'], b['h']
    return {'l': (x - w / 2, y - h / 2 + frac * h), 'r': (x + w / 2, y - h / 2 + frac * h),
            't': (x - w / 2 + frac * w, y + h / 2), 'b': (x - w / 2 + frac * w, y - h / 2)}[where]


def arrow(p0, p1, ls=BOTH, lw=0.6, color=EDGE, rad=0.0, label=None, lpos=0.5, loff=(0, 0.05),
          ha='center'):
    """
    Arrow from p0 to p1 (data units are inches on both axes).

    The shaft carries the line style (dashed/dotted for one mode); the head is
    always a solid filled triangle, so dashed or dotted arrows keep a crisp tip.
    rad bends the shaft like matplotlib's arc3 connection.
    """
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    if rad:

        c = (p0 + p1) / 2 + rad * np.array([d[1], -d[0]])
        t = np.linspace(0.0, 1.0, 120)[:, None]
        pts = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * c + t ** 2 * p1
        tangent = p1 - c
    else:
        pts = np.vstack([p0, p1])
        tangent = d
    u = tangent / np.hypot(*tangent)
    n = np.array([-u[1], u[0]])
    head_len = 0.055 + 0.03 * (lw - 0.6)
    head_w = 0.6 * head_len
    base = p1 - head_len * u

    seg = np.hypot(*np.diff(pts, axis=0).T)
    from_end = np.concatenate([np.cumsum(seg[::-1])[::-1], [0.0]])
    keep = from_end >= 0.8 * head_len
    shaft = np.vstack([pts[keep], p1 - 0.8 * head_len * u])
    ax.plot(shaft[:, 0], shaft[:, 1], color=color, lw=lw, ls=ls, solid_capstyle='butt',
            dash_capstyle='butt', zorder=2)
    ax.add_patch(Polygon([p1, base + head_w / 2 * n, base - head_w / 2 * n], closed=True,
                         fc=color, ec=color, lw=0.2, joinstyle='miter', zorder=3))
    if label:
        lx = p0[0] + lpos * (p1[0] - p0[0]) + loff[0]
        ly = p0[1] + lpos * (p1[1] - p0[1]) + loff[1]
        ax.text(lx, ly, label, ha=ha, va='center', fontsize=8, color=style.INK_2,
                bbox=dict(fc='white', ec='none', pad=0.6) if loff == (0, 0) else None)


group(0.07, 1.34, 1.30, 2.21, 'Application nodes')
group(0.07, 0.14, 1.30, 1.19, 'Map and pose')
group(1.84, 0.24, 5.40, 2.21, 'Nav2 navigation servers (one parameter file, both modes)')

loc = box(0.685, 1.925, 1.07, 0.19, 'Global localizer', ls=NAVIGATION, lw=0.9)
wpr = box(0.685, 1.70, 1.07, 0.19, 'Waypoint runner', ls=NAVIGATION, lw=0.9)
exp = box(0.685, 1.475, 1.07, 0.19, 'Frontier explorer', ls=MAPPING)

slam = box(0.73, 0.93, 1.06, 0.22, 'SLAM Toolbox', ls=MAPPING)
msrv = box(0.73, 0.67, 1.06, 0.16, 'Map server', ls=NAVIGATION, lw=0.9)
amcl = box(0.73, 0.40, 1.06, 0.16, 'AMCL', ls=NAVIGATION, lw=0.9)
keep = box(0.73, 0.215, 1.06, 0.13, 'Keepout mask servers')

bt = box(2.50, 1.66, 1.05, 0.30, 'Behavior-tree\nnavigator')
cost = box(2.50, 0.62, 1.05, 0.42, 'Global and local\ncostmaps with\nkeepout filter')
rec = box(3.88, 1.86, 1.00, 0.30, 'Recovery\nbehaviors')
plan = box(3.88, 1.24, 1.00, 0.30, 'Planner\n(NavFn, A*)')
ctrl = box(3.88, 0.62, 1.00, 0.30, 'Controller\n(MPPI, 20 Hz)')
smo = box(4.96, 1.24, 0.74, 0.30, 'Velocity\nsmoother')
mon = box(4.96, 0.62, 0.74, 0.30, 'Collision\nmonitor')

gz = box(6.52, 0.93, 1.16, 0.62, 'Gazebo Harmonic\nrobot model, LIDAR,\ncamera, odometry,\nworld', lw=0.7)


arrow(side(wpr, 'r'), side(bt, 'l', 0.5), ls=NAVIGATION, lw=0.9)
arrow(side(exp, 'r'), side(bt, 'l', 0.12), ls=MAPPING)
ax.text(1.57, 1.59, 'pose goals', ha='center', va='center', fontsize=8, color=style.INK_2)
arrow(side(loc, 'r'), side(rec, 'l', 0.6), ls=NAVIGATION, lw=0.9, label='spin', lpos=0.42,
      loff=(0, 0.06))


x_lane, y_link = 0.022, amcl['y'] - amcl['h'] / 2 + 0.72 * amcl['h']
ax.plot([x_lane, x_lane], [y_link, loc['y']], color=EDGE, lw=0.9, ls=NAVIGATION)
arrow((x_lane, loc['y']), side(loc, 'l'), ls=NAVIGATION, lw=0.9)
arrow((x_lane, y_link), side(amcl, 'l', 0.72), ls=NAVIGATION, lw=0.9)
ax.text(0.05, 1.265, 're-initialize, pose', ha='left', va='center', fontsize=8,
        color=style.INK_2)


x_map = side(exp, 'b', 0.82)[0]
arrow((x_map, slam['y'] + slam['h'] / 2), side(exp, 'b', 0.82), ls=MAPPING)
ax.text(x_map + 0.05, 1.265, 'map, pose', ha='left', va='center', fontsize=8,
        color=style.INK_2)
arrow(side(slam, 'r'), side(cost, 'l', 0.9), ls=MAPPING, label='map,\npose', lpos=0.42,
      loff=(0, 0.16))
arrow(side(msrv, 'r'), side(cost, 'l', 0.62), ls=NAVIGATION, lw=0.9, label='map', lpos=0.5,
      loff=(0, 0.055))
arrow(side(amcl, 'r'), side(cost, 'l', 0.34), ls=NAVIGATION, lw=0.9, label='pose', lpos=0.5,
      loff=(0, 0.075))
arrow(side(keep, 'r'), side(cost, 'l', 0.06), label='mask', lpos=0.5, loff=(0, -0.075))

arrow(side(msrv, 'b', 0.5), side(amcl, 't', 0.5), ls=NAVIGATION, lw=0.9)
ax.text(msrv['x'] + 0.05, (msrv['y'] + amcl['y']) / 2, 'map', ha='left', va='center',
        fontsize=8, color=style.INK_2)

arrow(side(bt, 'r', 0.8), side(rec, 'l', 0.3))

arrow(side(bt, 'r', 0.5), side(plan, 'l', 0.85))
arrow(side(plan, 'l', 0.55), side(bt, 'r', 0.2))
ax.text(3.25, 1.575, 'goal', ha='left', va='center', fontsize=8, color=style.INK_2)
ax.text(3.16, 1.36, 'path', ha='right', va='center', fontsize=8, color=style.INK_2)
arrow(side(bt, 'b', 0.75), side(ctrl, 'l', 0.8), rad=0.12)

ax.text(2.86, 1.13, 'path', ha='right', va='center', fontsize=8, color=style.INK_2)
arrow(side(cost, 'r', 0.75), side(plan, 'l', 0.25))
arrow(side(cost, 'r', 0.45), side(ctrl, 'l', 0.45))


arrow(side(ctrl, 'r', 0.75), side(smo, 'l', 0.25), lw=1.2, color=style.INK)
arrow(side(rec, 'r', 0.4), side(smo, 't', 0.5), rad=-0.3, lw=1.2, color=style.INK)
ax.text(4.72, 1.60, 'velocity', ha='right', va='center', fontsize=8, color=style.INK_2)
arrow(side(smo, 'b', 0.5), side(mon, 't', 0.5), lw=1.2, color=style.INK)
arrow(side(mon, 'r', 0.75), side(gz, 'l', 0.38), lw=1.2, color=style.INK)
ax.text(5.67, 0.90, 'velocity\ncommand', ha='center', va='bottom', fontsize=8,
        color=style.INK_2, linespacing=1.0)
arrow(side(gz, 'l', 0.12), side(mon, 'r', 0.2))
ax.text(5.67, 0.62, 'scan', ha='center', va='top', fontsize=8, color=style.INK_2)


x_down, y_bus, x_feed = gz['x'], 0.07, 0.115
ax.plot([x_down, x_down], [gz['y'] - gz['h'] / 2, y_bus], color=EDGE, lw=0.6)
ax.plot([x_down, x_feed], [y_bus, y_bus], color=EDGE, lw=0.6)
ax.plot([x_feed, x_feed], [y_bus, slam['y']], color=EDGE, lw=0.6)

arrow((x_feed, slam['y']), side(slam, 'l'))
arrow((x_feed, side(amcl, 'l', 0.28)[1]), side(amcl, 'l', 0.28))
arrow((cost['x'], y_bus), side(cost, 'b'))
ax.text(4.0, y_bus + 0.03, 'scan, wheel odometry', ha='center', va='bottom',
        fontsize=8, color=style.INK_2)


lx, ly = 5.80, 2.08
for i, (ls, lw, text) in enumerate(((BOTH, 0.6, 'both modes'), (MAPPING, 0.6, 'mapping only'),
                                    (NAVIGATION, 0.9, 'navigation only'))):
    y = ly - i * 0.16
    ax.plot([lx, lx + 0.32], [y, y], color=EDGE, lw=lw, ls=ls)
    ax.text(lx + 0.40, y, text, ha='left', va='center', fontsize=8, color=style.INK_2)

fig.savefig(style.FIGURES / 'fig_architecture.pdf')
print('wrote', style.FIGURES / 'fig_architecture.pdf')
