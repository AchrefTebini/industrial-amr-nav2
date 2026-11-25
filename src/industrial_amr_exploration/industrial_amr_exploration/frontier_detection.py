"""
Frontier detection on occupancy grids.

A frontier cell is a free cell with at least one unknown 4-neighbour.  Frontier
cells are grouped into 8-connected clusters.  A cluster's goal is the cluster
cell closest to the cluster centroid (always a free cell, unlike the centroid
itself), skipping cells too close to the robot: right after start-up the only
frontier is often a ring around the robot, whose centroid is the robot itself.
"""

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

UNKNOWN = -1
FREE = 0


@dataclass(frozen=True)
class GridInfo:
    """Placement of an occupancy grid in the world (nav_msgs/MapMetaData subset)."""

    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def cell_to_world(self, row, col):
        """Return the world (x, y) of a cell centre."""
        lx = (col + 0.5) * self.resolution
        ly = (row + 0.5) * self.resolution
        c, s = np.cos(self.origin_yaw), np.sin(self.origin_yaw)
        return self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly

    def world_to_cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = np.cos(self.origin_yaw), np.sin(self.origin_yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return int(np.floor(ly / self.resolution)), int(np.floor(lx / self.resolution))


@dataclass(frozen=True, eq=False)
class Frontier:
    """A cluster of frontier cells, in world coordinates."""

    xs: np.ndarray
    ys: np.ndarray

    @property
    def size(self):
        return self.xs.size

    def goal(self, robot_x, robot_y, min_distance=0.0):
        """
        Return the (x, y) goal for this frontier, or None.

        The goal is the cell closest to the centroid among the cells at least
        ``min_distance`` away from the robot.
        """
        far_enough = np.hypot(self.xs - robot_x, self.ys - robot_y) >= min_distance
        if not far_enough.any():
            return None
        to_centroid = (self.xs - self.xs.mean()) ** 2 + (self.ys - self.ys.mean()) ** 2
        i = int(np.argmin(np.where(far_enough, to_centroid, np.inf)))
        return float(self.xs[i]), float(self.ys[i])


def frontier_mask(grid, occupied_clearance_cells=0):
    """
    Return a boolean mask of frontier cells.

    ``grid`` is a 2D int array (rows = y, cols = x): -1 unknown, 0 free,
    >0 occupied.  Frontier cells closer than ``occupied_clearance_cells`` to an
    occupied cell are dropped; such goals are usually unreachable.

    """
    free = grid == FREE
    unknown = grid == UNKNOWN

    # Pad with "known" so cells on the border have no phantom unknown
    # neighbours (np.roll would wrap around to the opposite edge).
    padded = np.pad(unknown, 1, constant_values=False)
    unknown_neighbour = (
        padded[:-2, 1:-1] | padded[2:, 1:-1] | padded[1:-1, :-2] | padded[1:-1, 2:]
    )
    mask = free & unknown_neighbour

    if occupied_clearance_cells > 0:
        occupied = grid > FREE
        if occupied.any():
            dist_to_occupied = ndimage.distance_transform_edt(~occupied)
            mask &= dist_to_occupied >= occupied_clearance_cells
    return mask


def find_frontiers(grid, info, min_size_cells=1, occupied_clearance_cells=0):
    """
    Cluster frontier cells and return one Frontier per cluster.

    Clusters smaller than ``min_size_cells`` are discarded as SLAM noise.
    """
    mask = frontier_mask(grid, occupied_clearance_cells)
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
    if count == 0:
        return []

    # One sort + split groups the cells by cluster. A `labels == k` mask per
    # cluster would rescan the whole map once for every cluster.
    rows, cols = np.nonzero(labels)
    ids = labels[rows, cols]
    order = np.argsort(ids, kind='stable')
    rows, cols, ids = rows[order], cols[order], ids[order]
    splits = np.flatnonzero(np.diff(ids)) + 1

    frontiers = []
    for r, c in zip(np.split(rows, splits), np.split(cols, splits)):
        if r.size < min_size_cells:
            continue
        frontiers.append(Frontier(*info.cell_to_world(r, c)))
    return frontiers
