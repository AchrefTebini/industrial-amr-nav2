"""
World geometry for the evaluation, read from the repo's own files.

- Obstacles that cut the LIDAR scan plane, parsed from the world SDF (boxes
  and cylinders of static models, walls included).
- The charger base plate (below the scan plane) and the keepout zone, read
  from the keepout mask the costmaps use.
- The robot footprint from nav2_params.yaml (a run's own snapshot when it has
  one), as (xmin, xmax, ymin, ymax) in the robot frame: it need not be centred
  on the robot frame.

Distances are planar (x, y) in the world frame, which is also the map frame
because the robot spawns at the origin.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

WS = Path(__file__).resolve().parents[2]
SDF = WS / 'src/industrial_amr_gazebo/worlds/industrial_room.sdf'
KEEPOUT_YAML = WS / 'src/industrial_amr_navigation/maps/charger_keepout.yaml'
NAV2_PARAMS = WS / 'src/industrial_amr_navigation/config/nav2_params.yaml'
PROPERTIES = WS / 'src/industrial_amr_description/urdf/properties.xacro'

ROOM_X = (-5.0, 5.0)    # inner wall faces (industrial_room.sdf header comment)
ROOM_Y = (-4.0, 4.0)
ROOM_AREA = (ROOM_X[1] - ROOM_X[0]) * (ROOM_Y[1] - ROOM_Y[0])


def scan_plane_height():
    props = {}
    root = ET.parse(PROPERTIES).getroot()
    for el in root:
        if el.tag.endswith('property') and el.get('value') is not None:
            props[el.get('name')] = el.get('value')
    return (float(props['wheel_radius']) + float(props['chassis_height'])
            + float(props['lidar_length']) / 2.0)


@dataclass(frozen=True)
class Box:
    name: str
    cx: float
    cy: float
    sx: float
    sy: float
    yaw: float

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.sx / 2, self.sy / 2
        local = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
        return np.array([(self.cx + c * x - s * y, self.cy + s * x + c * y) for x, y in local])

    def signed_distance(self, px, py):
        """Signed distance from points to the box boundary (negative inside)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = np.asarray(px) - self.cx, np.asarray(py) - self.cy
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        qx, qy = np.abs(lx) - self.sx / 2, np.abs(ly) - self.sy / 2
        outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
        inside = np.minimum(np.maximum(qx, qy), 0)
        return outside + inside


@dataclass(frozen=True)
class Circle:
    name: str
    cx: float
    cy: float
    r: float

    def signed_distance(self, px, py):
        return np.hypot(np.asarray(px) - self.cx, np.asarray(py) - self.cy) - self.r


def _floats(text):
    return [float(v) for v in text.split()]


def load_world():
    """Return (scan-plane obstacles, charger plate box) from the SDF."""
    plane = scan_plane_height()
    obstacles, plate = [], None
    for model in ET.parse(SDF).getroot().iter('model'):
        name = model.get('name')
        if name == 'ground_plane' or model.findtext('static', '').strip() != 'true':
            continue
        x, y, z, _, _, yaw = _floats(model.findtext('pose'))
        geom = model.find('link/collision/geometry')
        box, cyl = geom.find('box'), geom.find('cylinder')
        if box is not None:
            sx, sy, sz = _floats(box.findtext('size'))
            shape = Box(name, x, y, sx, sy, yaw)
        else:
            r, sz = float(cyl.findtext('radius')), float(cyl.findtext('length'))
            shape = Circle(name, x, y, r)
        if name == 'charger_base':
            plate = shape
        if z - sz / 2 <= plane <= z + sz / 2:
            obstacles.append(shape)
    return obstacles, plate


def keepout_zone():
    """Bounding box (x0, x1, y0, y1) of the lethal cells of the keepout mask."""
    meta = yaml.safe_load(KEEPOUT_YAML.read_text())
    img = read_pgm(KEEPOUT_YAML.parent / meta['image'])
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    rows, cols = np.nonzero(img == 0)       # trinary, negate 0: black = occupied
    h = img.shape[0]
    x0, x1 = ox + cols.min() * res, ox + (cols.max() + 1) * res
    y0, y1 = oy + (h - rows.max() - 1) * res, oy + (h - rows.min()) * res
    return x0, x1, y0, y1


def keepout_box():
    x0, x1, y0, y1 = keepout_zone()
    return Box('keepout_zone', (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.0)


def footprint_bounds(params_path=NAV2_PARAMS):
    """(xmin, xmax, ymin, ymax) of the rectangular Nav2 footprint in the robot frame.

    params_path is a nav2_params.yaml; pass a run's own snapshot so that data
    recorded with another footprint are evaluated with the one they used.
    """
    params = yaml.safe_load(Path(params_path).read_text())
    pts = yaml.safe_load(params['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def run_footprint(run_dir, default=None):
    """Footprint bounds a run was recorded with.

    Order: the run's own nav2_params.yaml snapshot (written by run_experiments.py),
    then `default` (a nav2_params.yaml path), then the repo file. Runs recorded
    before snapshots existed (paper/data) fall through to the last two; a note
    on stderr says which file was used.
    """
    snap = Path(run_dir) / 'nav2_params.yaml'
    if snap.exists():
        return footprint_bounds(snap)
    source = Path(default) if default else NAV2_PARAMS
    print(f'note: {run_dir} has no nav2_params.yaml snapshot; footprint from {source}',
          file=sys.stderr)
    return footprint_bounds(source)


def footprint_half_extents():
    """(half length, half width) of a footprint centred on the robot frame."""
    xmin, xmax, ymin, ymax = footprint_bounds()
    if xmin != -xmax or ymin != -ymax:
        raise ValueError('footprint is not centred on the robot frame; use footprint_bounds()')
    return xmax, ymax


def _as_bounds(fp):
    """Accept (half length, half width) or (xmin, xmax, ymin, ymax)."""
    if fp is None:
        return footprint_bounds()
    if len(fp) == 2:
        return -fp[0], fp[0], -fp[1], fp[1]
    return tuple(fp)


def footprint_box(x, y, yaw, half=None):
    """The footprint at robot pose (x, y, yaw) as a Box.

    half is (half length, half width) for a centred footprint, or
    (xmin, xmax, ymin, ymax) for one offset from the robot frame.
    """
    xmin, xmax, ymin, ymax = _as_bounds(half)
    ox, oy = (xmin + xmax) / 2, (ymin + ymax) / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return Box('footprint', x + c * ox - s * oy, y + s * ox + c * oy, xmax - xmin, ymax - ymin,
               yaw)


def footprint_clearance(x, y, yaw, shape, half=None, step=0.01):
    """Minimum distance between the footprint at (x, y, yaw) and a shape (<= 0: overlap)."""
    fp = footprint_box(x, y, yaw, half)
    if isinstance(shape, Circle):
        return float(fp.signed_distance(shape.cx, shape.cy) - shape.r)
    # Convex polygons: the closest pair always involves a vertex of one of them.
    corners = fp.corners()
    edges = []
    for a, b in zip(corners, np.roll(corners, -1, axis=0)):
        n = max(2, int(np.hypot(*(b - a)) / step))
        t = np.linspace(0, 1, n, endpoint=False)[:, None]
        edges.append(a + t * (b - a))
    perimeter = np.vstack(edges)
    d1 = shape.signed_distance(perimeter[:, 0], perimeter[:, 1]).min()
    oc = shape.corners()
    d2 = fp.signed_distance(oc[:, 0], oc[:, 1]).min()
    return float(min(d1, d2))


def read_pgm(path):
    data = Path(path).read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[pos:pos + w * h], dtype=np.uint8).reshape(h, w)


def load_map_yaml(path):
    """Saved map -> (grid with -1/0/100, resolution, origin_x, origin_y); row 0 = lowest y."""
    meta = yaml.safe_load(Path(path).read_text())
    img = read_pgm(Path(path).parent / meta['image'])[::-1]
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[img == 254] = 0
    grid[img == 0] = 100
    return grid, meta['resolution'], meta['origin'][0], meta['origin'][1]


def nearest_surface_distance(px, py, shapes):
    d = np.full(np.shape(px), np.inf)
    for s in shapes:
        d = np.minimum(d, np.abs(s.signed_distance(px, py)))
    return d
