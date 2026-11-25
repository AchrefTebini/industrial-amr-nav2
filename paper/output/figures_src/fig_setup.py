#!/usr/bin/env python3
"""
Fig. 1: the simulation setup, from Gazebo screenshots.

Usage (ROS and the workspace sourced, nothing else running):
  python3 fig_setup.py capture   # start the sim, take the raw screenshots
  python3 fig_setup.py compose   # crop and compose fig_setup.png from them

capture: sim.launch.py headless:=true (server, robot spawned at the origin),
plus a Gazebo GUI client on $DISPLAY (default :1) with gui_screenshot.config. The camera
is placed through the GUI's /gui/move_to/pose service and each view is saved
through /gui/screenshot. Raw images go to paper/data/screenshots/.
compose: (a) room overview, (b) robot close-up, labels drawn in the image,
3.5 in wide at 600 dpi.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
RAW = HERE.parents[2] / 'paper' / 'data' / 'screenshots'

VIEWS = {
    'overview': (0.0, -6.4, 12.6, 0.0, 1.08, math.pi / 2),
    'robot': (0.80, -0.68, 0.42, 0.0, 0.30, 2.44),
}


ANNOTATIONS = {
    'robot': [
        ('2D LIDAR', (1030, 350), (1160, 290)),
        ('RGB camera', (1150, 468), (1165, 615)),
        ('Drive wheel', (650, 470), (700, 588)),
        ('Caster', (955, 600), (862, 652)),
    ],
}


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def gz_service(service, reqtype, req, reptype='gz.msgs.Boolean', timeout=5000):
    return subprocess.run(['gz', 'service', '-s', service, '--reqtype', reqtype,
                           '--reptype', reptype, '--timeout', str(timeout), '--req', req],
                          capture_output=True, text=True)


def _child():
    os.setsid()
    signal.signal(signal.SIGINT, signal.SIG_DFL)


def stop(proc):
    os.kill(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        pass
    pids = subprocess.run(['pgrep', '-s', str(proc.pid)], capture_output=True,
                          text=True).stdout.split()
    for pid in pids:
        try:
            os.kill(int(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass


def capture():
    RAW.mkdir(parents=True, exist_ok=True)
    log = open(RAW / 'capture.log', 'w')
    sim = subprocess.Popen(['ros2', 'launch', 'industrial_amr_gazebo', 'sim.launch.py',
                            'headless:=true'], stdout=log, stderr=subprocess.STDOUT,
                           preexec_fn=_child)
    time.sleep(12)
    env = dict(os.environ, DISPLAY=os.environ.get('DISPLAY', ':1'))
    gui = subprocess.Popen(['gz', 'sim', '-g', '--gui-config', str(HERE / 'gui_screenshot.config')],
                           stdout=log, stderr=subprocess.STDOUT, env=env, preexec_fn=_child)
    try:
        time.sleep(15)
        for name, (x, y, z, r, p, yw) in VIEWS.items():
            qx, qy, qz, qw = quat(r, p, yw)
            req = (f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
                   f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}')
            res = gz_service('/gui/move_to/pose', 'gz.msgs.GUICamera', req)
            print(name, 'move_to:', res.stdout.strip(), res.stderr.strip())
            time.sleep(4)
            before = set(RAW.glob('*.png'))
            res = gz_service('/gui/screenshot', 'gz.msgs.StringMsg', f'data: "{RAW}"')
            print(name, 'screenshot:', res.stdout.strip(), res.stderr.strip())
            time.sleep(3)
            new = sorted(set(RAW.glob('*.png')) - before, key=lambda q: q.stat().st_mtime)
            if new:
                new[-1].rename(RAW / f'{name}.png')
                print('saved', RAW / f'{name}.png')
            else:
                print('no screenshot for', name)
    finally:
        stop(gui)
        stop(sim)


def compose(crops):
    sys.path.insert(0, str(HERE))
    import matplotlib.pyplot as plt
    from PIL import Image
    import style

    style.apply()
    fig, axes = plt.subplots(1, 2, figsize=(style.COLUMN_W, 1.05),
                             gridspec_kw=dict(width_ratios=[crops['overview'][2] /
                                                            crops['overview'][3],
                                                            crops['robot'][2] /
                                                            crops['robot'][3]],
                                              wspace=0.04, left=0.0, right=1.0, top=1.0,
                                              bottom=0.0))
    for ax, name, label in zip(axes, ('overview', 'robot'), ('(a)', '(b)')):
        x, y, w, h = crops[name]
        img = Image.open(RAW / f'{name}.png').convert('RGB').crop((x, y, x + w, y + h))
        ax.imshow(img)
        ax.set_axis_off()
        for text, target, where in ANNOTATIONS.get(name, []):
            ax.annotate(text, xy=(target[0] - x, target[1] - y), xytext=(where[0] - x, where[1] - y),
                        fontsize=8, color=style.INK, ha='center', va='center',
                        bbox=dict(fc='white', ec='none', pad=0.6, alpha=0.9),
                        arrowprops=dict(arrowstyle='-', lw=0.5, color=style.INK,
                                        shrinkA=0, shrinkB=1))
        ax.text(0.02, 0.97, label, transform=ax.transAxes, ha='left', va='top', fontsize=8,
                color=style.INK, bbox=dict(fc='white', ec='none', pad=0.8, alpha=0.85))
    out = style.FIGURES / 'fig_setup.png'
    fig.savefig(out, dpi=600, pad_inches=0)
    print('wrote', out)


if __name__ == '__main__':
    if sys.argv[1:] == ['capture']:
        capture()
    elif sys.argv[1:2] == ['compose']:
        import json
        compose(json.loads(sys.argv[2]))
    else:
        raise SystemExit(__doc__)
