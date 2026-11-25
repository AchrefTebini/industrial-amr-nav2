#!/usr/bin/env python3
"""
Paste-ready LaTeX tables (output/tables/*.tex).

TABLE I and II restate values from the repo; a few are re-read from the source
files below so the tables fail loudly if the configs change. TABLE III and IV are
built from paper/data/results.
Style: caption above, \\hline rules, units in headers, footnotes a, b.
"""

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

WS = Path(__file__).resolve().parents[3]
TABLES = WS / 'paper' / 'output' / 'tables'
RESULTS = WS / 'paper' / 'data' / 'results'
SRC = WS / 'src'


def xacro_props():
    root = ET.parse(SRC / 'industrial_amr_description/urdf/properties.xacro').getroot()
    return {el.get('name'): el.get('value') for el in root if el.tag.endswith('property')}


def check_sources():
    """Guard: the hard-coded table values must still match the repo."""
    p = xacro_props()
    assert (p['chassis_length'], p['chassis_width'], p['chassis_height']) == ('0.60', '0.40', '0.15')
    assert (p['wheel_radius'], p['lidar_range_min'], p['lidar_range_max']) == ('0.075', '0.12', '12.0')
    nav = yaml.safe_load((SRC / 'industrial_amr_navigation/config/nav2_params.yaml').read_text())
    vs = nav['velocity_smoother']['ros__parameters']
    assert vs['max_velocity'] == [0.5, 0.0, 1.5] and vs['min_velocity'] == [-0.35, 0.0, -1.5]
    assert vs['max_accel'] == [1.5, 0.0, 2.5]
    gc = nav['controller_server']['ros__parameters']['general_goal_checker']
    assert (gc['xy_goal_tolerance'], gc['yaw_goal_tolerance']) == (0.20, 0.15)
    infl = nav['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
    assert infl['inflation_radius'] == 0.55
    fe = yaml.safe_load((SRC / 'industrial_amr_exploration/config/frontier_explorer.yaml')
                        .read_text())['frontier_explorer']['ros__parameters']
    assert (fe['min_goal_distance'], fe['obstacle_clearance'], fe['goal_timeout'],
            fe['empty_checks_to_finish']) == (0.5, 0.35, 30.0, 3)


def write(name, text):
    TABLES.mkdir(parents=True, exist_ok=True)
    (TABLES / name).write_text(text)
    print('wrote', TABLES / name)


def table_robot():
    rows = [
        ('Chassis length, width, height (m)', '0.60, 0.40, 0.15'),
        ('Navigation footprint (m)', r'0.60 $\times$ 0.50'),
        ('Total mass (kg)', '11.5'),
        ('Drive wheel radius, separation (m)', '0.075, 0.45'),
        ('Front casters, radius (m)', '2, 0.035'),
        ('Wheel odometry rate (Hz)', '50'),
        ('LIDAR beams, field of view', r'360, 360$^\circ$'),
        ('LIDAR range (m), rate (Hz)', r'0.12--12, 10'),
        (r'LIDAR noise $\sigma$, scan height (m)', '0.01, 0.245'),
        ('Camera resolution, field of view', r'640 $\times$ 480, 80$^\circ$'),
        ('Camera rate (Hz)', '30'),
        ('Linear velocity (m/s)', r'$-0.35$ to 0.5'),
        ('Angular velocity (rad/s)', r'$\pm 1.5$'),
        ('Acceleration limits\\textsuperscript{a}', r'1.5~m/s$^2$, 2.5~rad/s$^2$'),
    ]
    body = '\n'.join(f'{a} & {b} \\\\' for a, b in rows)
    write('tab_robot.tex', rf"""% TABLE I: robot and sensor specifications
\begin{{table}}[t]
\caption{{Simulated Robot and Sensor Specifications}}
\label{{tab:robot}}
\centering
\footnotesize
\begin{{tabular}}{{|l|l|}}
\hline
\textbf{{Parameter}} & \textbf{{Value}} \\
\hline
{body}
\hline
\multicolumn{{2}}{{l}}{{\textsuperscript{{a}}Set by the velocity smoother; the MPPI controller allows more.}}
\end{{tabular}}
\end{{table}}
""")


def table_design():
    rows = [
        ('Goal tolerance', r'0.20~m, 0.15~rad',
         r'A differential drive cannot correct a lateral offset directly; at 0.10~m the '
         r'robot crept for 30--40~s and failed the progress check'),
        ('Inflation radius', '0.55~m', r'Circumscribed radius (0.39~m) plus a margin'),
        ('Waypoint clearance', r'$\geq 0.8$~m',
         r'Inflation plus footprint; two waypoints were moved'),
        ('Minimum goal distance', '0.5~m',
         r'Start-up frontier is a ring centered on the robot; turning in place adds no map'),
        ('Frontier clearance', '0.35~m', r'Goals next to walls lie in inflated cost'),
        ('Goal timeout', '30~s', r'Backstop for the progress check (0.5~m in 10~s)'),
        ('Finish condition', r'3 empty checks', r'The SLAM map is briefly incomplete after updates'),
        ('Keepout margin', '0.3~m', r'Costmap filters act after inflation'),
    ]
    body = '\n'.join(f'{a} & {b} & {c} \\\\ \\hline' for a, b, c in rows)
    write('tab_design.tex', rf"""% TABLE II: design decisions
\begin{{table}}[t]
\caption{{Tuned Parameters and the Reasons Behind Them}}
\label{{tab:design}}
\centering
\footnotesize
\begin{{tabular}}{{|p{{0.78in}}|p{{0.62in}}|p{{1.55in}}|}}
\hline
\textbf{{Parameter}} & \textbf{{Value}} & \textbf{{Reason}} \\
\hline
{body}
\end{{tabular}}
\end{{table}}
""")


def num(v, digits):
    """Number with a proper minus sign (math mode) when negative."""
    text = f'{v:.{digits}f}'
    return f'${text}$' if text.startswith('-') else text


def pm(s, digits=1, scale=1.0):
    return f"{num(s['mean'] * scale, digits)} $\\pm$ {num(s['std'] * scale, digits)}"


def rng(s, digits=1, scale=1.0):
    return num(s['min'] * scale, digits), num(s['max'] * scale, digits)


def table_mapping():
    summary = json.loads((RESULTS / 'mapping_summary.json').read_text())
    m = summary['metrics']
    rows = [
        ('Exploring time (s)', 't_explore', 1, 1),
        ('Return to start (s)\\textsuperscript{b}', 't_return', 1, 1),
        ('Total time (s)\\textsuperscript{b}', 't_total', 1, 1),
        ('Path length (m)', 'path_length', 1, 1),
        ('Explored area (m$^2$)', 'area_known', 2, 1),
        ('Room coverage (\\%)', 'coverage', 1, 100),
        ('Map error, mean (cm)', 'err_mean', 1, 100),
        ('Map error, RMS (cm)', 'err_rms', 1, 100),
        ('Width error (cm)\\textsuperscript{c}', 'width_err', 1, 100),
        ('Length error (cm)\\textsuperscript{c}', 'length_err', 1, 100),
        ('Frontier goals sent', 'goals', 1, 1),
        ('Progress-check failures', 'progress_failures', 1, 1),
        ('Goal timeouts', 'goal_timeouts', 1, 1),
    ]
    lines = []
    counts = {'goals', 'progress_failures', 'goal_timeouts'}
    for label, key, d, sc in rows:
        s = m[key]
        lo, hi = rng(s, 0 if key in counts else d, sc)
        lines.append(f'{label} & {pm(s, d, sc)} & {lo} & {hi} \\\\')
    n = summary['runs']
    rtf = m['rtf']
    write('tab_mapping.tex', rf"""% TABLE III: mapping results (source: paper/data/results/mapping_summary.json)
\begin{{table}}[t]
\caption{{Autonomous Mapping Results Over {n} Runs\textsuperscript{{a}}}}
\label{{tab:mapping}}
\centering
\footnotesize
\begin{{tabular}}{{|l|c|c|c|}}
\hline
\textbf{{Metric}} & \textbf{{Mean $\pm$ std}} & \textbf{{Min}} & \textbf{{Max}} \\
\hline
{chr(10).join(lines)}
\hline
\multicolumn{{4}}{{p{{3.2in}}}}{{\textsuperscript{{a}}Simulation, real-time factor {rtf['mean']:.2f}. \textsuperscript{{b}}Over the {summary['completed']} runs that returned to the start. \textsuperscript{{c}}Measured minus true wall-to-wall distance (10~m, 8~m); the map resolution is 5~cm.}}
\end{{tabular}}
\end{{table}}
""")


def table_navigation():
    s = json.loads((RESULTS / 'nav_summary.json').read_text())
    lines = []
    for k, w in s['per_waypoint'].items():
        lines.append(
            f"WP{k} & {w['succeeded']}/{w['n']} & {w['time']['mean']:.1f} & "
            f"{w['path_length']['mean']:.2f} & {100 * w['pos_err']['mean']:.1f} $\\pm$ "
            f"{100 * w['pos_err']['std']:.1f} & "
            f"{math.degrees(w['yaw_err']['mean']):.1f} $\\pm$ "
            f"{math.degrees(w['yaw_err']['std']):.1f} \\\\")
    o = s['overall']
    lines.append('\\hline')
    lines.append(
        f"All & {s['succeeded']}/{s['goals']} & {o['time']['mean']:.1f} & "
        f"{o['path_length']['mean']:.2f} & {100 * o['pos_err']['mean']:.1f} $\\pm$ "
        f"{100 * o['pos_err']['std']:.1f} & {math.degrees(o['yaw_err']['mean']):.1f} $\\pm$ "
        f"{math.degrees(o['yaw_err']['std']):.1f} \\\\")
    write('tab_navigation.tex', rf"""% TABLE IV: navigation results (source: paper/data/results/nav_summary.json)
\begin{{table}}[t]
\caption{{Waypoint Navigation Results\textsuperscript{{a}}}}
\label{{tab:navigation}}
\centering
\footnotesize
\setlength{{\tabcolsep}}{{4pt}}
\begin{{tabular}}{{|c|c|c|c|c|c|}}
\hline
\textbf{{Goal}} & \textbf{{Success}} & \shortstack{{\textbf{{Time}}\\\textbf{{(s)}}}} & \shortstack{{\textbf{{Path}}\\\textbf{{(m)}}}} & \shortstack{{\textbf{{Position}}\\\textbf{{error (cm)}}}} & \shortstack{{\textbf{{Heading}}\\\textbf{{error ($^\circ$)}}}} \\
\hline
{chr(10).join(lines)}
\hline
\multicolumn{{6}}{{p{{3.3in}}}}{{\textsuperscript{{a}}{s['sessions']} sessions of 3 loops; final pose errors against Gazebo ground truth (mean $\pm$ std). AMCL position error over the missions: RMSE {100 * s['amcl_pos_rmse']:.1f}~cm, maximum {100 * s['amcl_pos_max']:.1f}~cm.}}
\end{{tabular}}
\end{{table}}
""")


if __name__ == '__main__':
    check_sources()
    which = sys.argv[1:] or ['robot', 'design', 'mapping', 'navigation']
    for w in which:
        globals()[f'table_{w}']()
