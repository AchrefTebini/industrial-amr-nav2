#!/usr/bin/env python3
"""
Spin test for the rotation-centre study (research/rotation_center/README.md).

Drives the robot through Nav2 to an open spot, then runs Nav2's Spin behaviour
by +2*pi and by -2*pi, with a pause before and after each spin. The recording
(ground truth, /odom, /tf) shows which point of the robot stays fixed while it
turns, and how far odometry drifts from the truth during the turn.

/rosout markers cut the recording into phases for the analysis:
  "approach start" / "approach done: <result>"
  "spin <k> start: <angle> rad" / "spin <k> done: <result>"

All motion goes through Nav2 actions (NavigateToPose, Spin); this node never
publishes velocity commands.

Usage (navigation.launch.py running; `run_experiments.py spin` does both):
  python3 paper/experiments/spin_test.py --ros-args -p use_sim_time:=true
Exit code 0 when the approach and both spins succeeded, 1 otherwise.
"""

import math
import sys

from industrial_amr_exploration.pose_utils import make_pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

# Open spot in the map frame (= world frame, the robot spawns at the origin).
# The nearest obstacle, charger plate or keepout zone (barrel 2) is 1.87 m
# from this point (world_geometry). While turning in place the body sweeps a
# 0.56 m radius around the drive axle, which is at most 0.20 m from the point,
# so at least 1.11 m of clearance remains in either frame convention.
SPOT = (2.8, 0.5, 0.0)
SPINS = (2.0 * math.pi, -2.0 * math.pi)
SETTLE = 2.0          # s of simulation time before and after each spin
SPIN_ALLOWANCE = 30   # s, Nav2 Spin time allowance (|wz| <= 1.0 rad/s in recoveries)


def wait_for_task(nav):
    while not nav.isTaskComplete():
        pass
    return nav.getResult()


def settle(nav, seconds):
    """Wait `seconds` of node time (simulation time with use_sim_time)."""
    t0 = nav.get_clock().now()
    while (nav.get_clock().now() - t0).nanoseconds < seconds * 1e9:
        rclpy.spin_once(nav, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator(node_name='spin_test')

    # Not waitUntilNav2Active(): with AMCL it publishes an initial pose at the
    # origin, which would overwrite the current localisation estimate.
    nav._waitForNodeToActivate('bt_navigator')
    nav._waitForNodeToActivate('behavior_server')

    ok = True
    x, y, yaw = SPOT
    nav.info(f'approach start: ({x:.2f}, {y:.2f}, {yaw:.2f})')
    nav.goToPose(make_pose(x, y, yaw, 'map', nav.get_clock().now().to_msg()))
    result = wait_for_task(nav)
    nav.info(f'approach done: {result.name}')
    if result != TaskResult.SUCCEEDED:
        ok = False
    else:
        settle(nav, SETTLE)
        for k, angle in enumerate(SPINS, start=1):
            nav.info(f'spin {k} start: {angle:+.4f} rad')
            nav.spin(spin_dist=angle, time_allowance=SPIN_ALLOWANCE)
            result = wait_for_task(nav)
            nav.info(f'spin {k} done: {result.name}')
            settle(nav, SETTLE)
            if result != TaskResult.SUCCEEDED:
                ok = False
                break

    nav.info(f'spin test {"passed" if ok else "FAILED"}')
    nav.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
