import math

from industrial_amr_exploration.frontier_detection import (
    find_frontiers, frontier_mask, GridInfo)
import numpy as np

U, F, OCC = -1, 0, 100


def test_no_frontiers_in_fully_known_map():
    grid = np.full((10, 10), F, dtype=np.int8)
    assert not frontier_mask(grid).any()


def test_free_cell_next_to_unknown_is_frontier():
    grid = np.array([[F, F, U],
                     [F, F, U],
                     [F, F, U]], dtype=np.int8)
    expected = np.array([[0, 1, 0],
                         [0, 1, 0],
                         [0, 1, 0]], dtype=bool)
    np.testing.assert_array_equal(frontier_mask(grid), expected)


def test_map_edges_do_not_wrap_around():
    # Unknown on the right border must not make the left border a frontier,
    # which is what np.roll's wrap-around would do.
    grid = np.full((5, 6), F, dtype=np.int8)
    grid[:, -1] = U
    mask = frontier_mask(grid)
    assert not mask[:, 0].any()
    assert mask[:, -2].all()


def test_obstacle_clearance_drops_cells_near_walls():
    grid = np.full((7, 7), F, dtype=np.int8)
    grid[:, -1] = U
    grid[0, :-1] = OCC
    mask = frontier_mask(grid, occupied_clearance_cells=2)
    assert not mask[1, -2]       # 1 cell from the wall
    assert mask[3, -2]


def test_clusters_are_split_and_small_ones_filtered():
    grid = np.full((20, 20), F, dtype=np.int8)
    grid[0:8, 19] = U        # 8 cells beside the strip + 1 below it
    grid[15:17, 0] = U       # 2-cell frontier along the left edge
    info = GridInfo(resolution=0.1, origin_x=0.0, origin_y=0.0)
    assert len(find_frontiers(grid, info, min_size_cells=1)) == 2
    big = find_frontiers(grid, info, min_size_cells=5)
    assert len(big) == 1 and big[0].size == 9


def test_goal_is_a_free_frontier_cell_in_world_coordinates():
    grid = np.full((10, 10), F, dtype=np.int8)
    grid[:, 9] = U
    info = GridInfo(resolution=0.5, origin_x=-2.0, origin_y=1.0)
    (frontier,) = find_frontiers(grid, info)
    gx, gy = frontier.goal(robot_x=-10.0, robot_y=-10.0)
    row, col = info.world_to_cell(gx, gy)
    assert col == 8 and grid[row, col] == F
    assert math.isclose(gx, -2.0 + 8.5 * 0.5)


def test_ring_frontier_around_robot_yields_goal_away_from_robot():
    # Right after start-up SLAM knows a disc around the robot: the frontier is
    # a ring whose centroid is the robot itself.  The goal must not be there.
    size, radius = 41, 12
    yy, xx = np.mgrid[:size, :size] - size // 2
    grid = np.where(xx ** 2 + yy ** 2 <= radius ** 2, F, U).astype(np.int8)
    info = GridInfo(resolution=0.1, origin_x=-2.05, origin_y=-2.05)  # robot at (0, 0)
    (ring,) = find_frontiers(grid, info)
    assert math.hypot(ring.xs.mean(), ring.ys.mean()) < 0.1
    gx, gy = ring.goal(0.0, 0.0, min_distance=0.5)
    assert math.hypot(gx, gy) >= 0.5


def test_no_goal_when_whole_frontier_is_too_close():
    grid = np.full((5, 5), U, dtype=np.int8)
    grid[2, 2] = F
    info = GridInfo(resolution=0.1, origin_x=-0.25, origin_y=-0.25)
    (frontier,) = find_frontiers(grid, info)
    assert frontier.goal(0.0, 0.0, min_distance=0.5) is None


def test_cell_world_round_trip_with_rotated_origin():
    info = GridInfo(resolution=0.05, origin_x=1.0, origin_y=-2.0, origin_yaw=0.7)
    for row, col in [(0, 0), (12, 40), (99, 3)]:
        assert info.world_to_cell(*info.cell_to_world(row, col)) == (row, col)
