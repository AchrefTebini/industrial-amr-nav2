#!/usr/bin/env python3
"""
Figures for the rotation-centre study, from results/ (run compare.py first).

Usage: python3 research/rotation_center/analysis/figures.py

Writes to research/rotation_center/figures/ (vector PDF with embedded fonts,
plus a 300 dpi PNG preview of each):
  fig_spin.pdf        (a) A and (b) B: true path of base_footprint and its
                      odometry estimate during the spins, in the robot frame
                      at spin start; (c) odometry error against the true
                      heading change, with the predicted 0.40|sin(dtheta/2)|
  fig_goal_error.pdf  final position error against ground truth per waypoint,
                      A0 / A / B, with the 0.20 m goal tolerance
"""

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

STUDY = Path(__file__).resolve().parents[1]
RESULTS = STUDY / 'results'
FIGURES = STUDY / 'figures'

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Nimbus Roman', 'TeX Gyre Termes', 'STIX Two Text',
                   'DejaVu Serif'],
    'mathtext.fontset': 'stix',
    'font.size': 8, 'axes.titlesize': 8, 'axes.labelsize': 8, 'legend.fontsize': 7,
    'xtick.labelsize': 7, 'ytick.labelsize': 7,
    'pdf.fonttype': 42, 'ps.fonttype': 42,
    'axes.linewidth': 0.6, 'lines.linewidth': 1.0,
})
STYLE = {'A0': {'color': '#999999', 'marker': 's', 'label': 'A0 (Oct 10)'},
         'A': {'color': '#E69F00', 'marker': 'o', 'label': 'A: frame at chassis centre'},
         'B': {'color': '#0072B2', 'marker': '^', 'label': 'B: frame on drive axle'}}


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def save(fig, name):
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURES / f'{name}.pdf', bbox_inches='tight')
    fig.savefig(FIGURES / f'{name}.png', dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {FIGURES / name}.pdf/.png')


def fig_spin():
    path = RESULTS / 'spin_curves.csv'
    if not path.exists():
        print('no spin_curves.csv; skipped fig_spin')
        return
    rows = read_csv(path)
    conds = [c for c in ('A', 'B') if any(r['condition'] == c for r in rows)]
    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.3),
                             gridspec_kw={'width_ratios': [1, 1, 1.5]})
    for ax, cond, tag in zip(axes[:2], ('A', 'B'), ('(a)', '(b)')):
        sel = [r for r in rows if r['condition'] == cond]
        spins = sorted({(r['session'], r['spin']) for r in sel})
        for i, key in enumerate(spins):
            pts = [r for r in sel if (r['session'], r['spin']) == key]
            gx = [float(r['gt_x']) for r in pts]
            gy = [float(r['gt_y']) for r in pts]
            ox = [float(r['odom_x']) for r in pts]
            oy = [float(r['odom_y']) for r in pts]
            ax.plot(gx, gy, '-', color=STYLE[cond]['color'], lw=0.8,
                    label=f'ground truth ({len(spins)} spins)' if i == 0 else None)
            ax.plot(ox[::40], oy[::40], 'x', color='black', ms=3.5, mew=0.6, ls='none',
                    label='odometry' if i == 0 else None)
        ax.plot([0], [0], 'o', mfc='none', mec='black', ms=5, mew=0.6)
        ax.set_xlim(-0.48, 0.12)
        ax.set_ylim(-0.30, 0.45)
        ax.set_aspect('equal')
        ax.set_xlabel('Forward (m)')
        ax.set_ylabel('Left (m)')
        ax.set_title(f'{tag} {"A" if cond == "A" else "B"}: frame at '
                     f'{"chassis centre" if cond == "A" else "drive axle"}', loc='left')
        if not sel:
            ax.text(0.5, 0.5, 'no data yet', transform=ax.transAxes, ha='center')
        else:
            ax.legend(loc='upper left', frameon=False, fontsize=6.5, handlelength=1.2,
                      borderaxespad=0.2)

    ax = axes[2]
    th = np.linspace(-2 * math.pi, 2 * math.pi, 400)
    ax.plot(th, 0.40 * np.abs(np.sin(th / 2)), '-', color='black', lw=0.8, zorder=3,
            label=r'predicted for A: $0.40\,|\sin(\Delta\theta/2)|$')
    for cond in conds:
        sel = [r for r in rows if r['condition'] == cond]
        ax.plot([float(r['dtheta']) for r in sel][::4], [float(r['odom_err']) for r in sel][::4],
                STYLE[cond]['marker'], color=STYLE[cond]['color'], ms=2.0, ls='none',
                label=f'measured, {cond}')
    ax.set_xlabel('True heading change (rad)')
    ax.set_ylabel('Odometry position error (m)')
    ax.set_xlim(-2 * math.pi, 2 * math.pi)
    ax.set_xticks([-2 * math.pi, -math.pi, 0, math.pi, 2 * math.pi])
    ax.set_xticklabels([r'$-2\pi$', r'$-\pi$', '0', r'$\pi$', r'$2\pi$'])
    ax.set_ylim(0, 0.62)
    ax.set_title('(c) Odometry error during the spins', loc='left')
    ax.legend(loc='upper center', frameon=False, ncol=1, handlelength=1.5,
              borderaxespad=0.2)
    fig.tight_layout(w_pad=1.0)
    save(fig, 'fig_spin')


def fig_goal_error():
    conds = [c for c in ('A0', 'A', 'B') if (RESULTS / f'{c}_nav_goals.csv').exists()]
    if not conds:
        print('no nav_goals; skipped fig_goal_error')
        return
    data = {c: [r for r in read_csv(RESULTS / f'{c}_nav_goals.csv') if r['success'] == 'True']
            for c in conds}
    waypoints = sorted({int(r['waypoint']) for c in conds for r in data[c]})
    fig, ax = plt.subplots(figsize=(3.5, 2.3))
    width = 0.8 / len(conds)
    rng = np.random.default_rng(0)
    for j, c in enumerate(conds):
        for k in waypoints:
            v = [float(r['pos_err']) for r in data[c] if int(r['waypoint']) == k]
            x0 = k - 0.4 + width * (j + 0.5)
            ax.plot(x0 + rng.uniform(-width / 4, width / 4, len(v)), v, STYLE[c]['marker'],
                    color=STYLE[c]['color'], ms=2.2, alpha=0.8, ls='none',
                    label=STYLE[c]['label'] if k == waypoints[0] else None)
            if v:
                ax.plot([x0 - width / 2.5, x0 + width / 2.5], [np.median(v)] * 2, '-',
                        color='black', lw=1.0)
    ax.axhline(0.20, color='black', ls='--', lw=0.7)
    ax.text(waypoints[-1] + 0.45, 0.205, 'tolerance', ha='right', va='bottom', fontsize=7)
    ax.set_xticks(waypoints)
    ax.set_xticklabels([f'WP{k}' for k in waypoints])
    ax.set_xlabel('Waypoint')
    ax.set_ylabel('Final position error (m)')
    ax.set_ylim(bottom=0)
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), frameon=False, ncol=len(conds),
              fontsize=6.5, handletextpad=0.2, columnspacing=0.8)
    fig.tight_layout()
    save(fig, 'fig_goal_error')


if __name__ == '__main__':
    fig_spin()
    fig_goal_error()
