#!/usr/bin/env python3
"""
Check that condition B's URDF is condition A's robot with only the reference
frame moved.

Usage: python3 check_urdf_shift.py A.urdf B.urdf [--dx 0.20]

Both files are rendered URDFs (e.g. the robot_description.urdf snapshot that
run_experiments.py writes into every run directory). The check passes when:

1. Every physical element (each visual, collision and inertial origin, and each
   link frame below base_link) sits at A's pose shifted by +dx in x, with the
   same orientation, relative to base_footprint (tolerance 1e-9 m / rad).
2. base_link itself does not move relative to base_footprint (it *is* the
   reference that moved with it).
3. Everything except origins is identical: geometry, masses, inertia tensors,
   joint axes and dynamics, materials, and the Gazebo sensor and plugin blocks
   (compared as canonical XML with every <origin> removed).

Exit code 0 when all checks pass, 1 otherwise.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

TOL = 1e-9


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_tf(el):
    """4x4 transform of an element's <origin> child (identity if absent)."""
    t = np.eye(4)
    o = el.find('origin') if el is not None else None
    if o is not None:
        t[:3, :3] = rpy_matrix(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        t[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return t


def poses(urdf_path):
    """Poses relative to base_footprint: link frames and visual/collision/inertial origins."""
    root = ET.parse(urdf_path).getroot()
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    links = {link.get('name'): link for link in root.findall('link')}

    frames = {}

    def frame(name):
        if name not in frames:
            if name == 'base_footprint':
                frames[name] = np.eye(4)
            else:
                j = joints[name]
                frames[name] = frame(j.find('parent').get('link')) @ origin_tf(j)
        return frames[name]

    out = {}
    for name, link in links.items():
        out[(name, 'frame', 0)] = frame(name)
        for kind in ('visual', 'collision', 'inertial'):
            for i, el in enumerate(link.findall(kind)):
                out[(name, kind, i)] = frame(name) @ origin_tf(el)
    return out


def without_origins(urdf_path):
    root = ET.parse(urdf_path).getroot()
    for parent in root.iter():
        for o in parent.findall('origin'):
            parent.remove(o)
    return ET.canonicalize(ET.tostring(root, encoding='unicode'), strip_text=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('a')
    parser.add_argument('b')
    parser.add_argument('--dx', type=float, default=0.20)
    args = parser.parse_args()

    pa, pb = poses(args.a), poses(args.b)
    ok = True
    if set(pa) != set(pb):
        print('element sets differ:', sorted(set(pa) ^ set(pb)))
        ok = False

    shift = np.eye(4)
    shift[0, 3] = args.dx
    print(f'{"element":42} {"A x, y, z (m)":>26} {"B x, y, z (m)":>26}  check')
    for key in sorted(set(pa) & set(pb)):
        ta, tb = pa[key], pb[key]
        fixed = key[0] in ('base_footprint', 'base_link') and key[1] == 'frame'
        expected = ta if fixed else shift @ ta
        good = np.allclose(tb, expected, atol=TOL, rtol=0)
        ok &= good
        label = f'{key[0]} {key[1]}' + (f' {key[2]}' if key[2] else '')
        fa = ', '.join(f'{v:+.4f}' for v in ta[:3, 3])
        fb = ', '.join(f'{v:+.4f}' for v in tb[:3, 3])
        what = 'unchanged' if fixed else f'+{args.dx:.2f} x'
        print(f'{label:42} {fa:>26} {fb:>26}  {what}: {"ok" if good else "FAIL"}')

    same = without_origins(args.a) == without_origins(args.b)
    print(f'non-origin content (geometry, inertia, joints, Gazebo blocks) identical: '
          f'{"ok" if same else "FAIL"}')
    ok &= same
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
