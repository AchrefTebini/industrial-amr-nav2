#!/usr/bin/env python3
"""
Evaluation only: log Gazebo ground truth straight from gz-transport.

Writes two CSV files until SIGINT/SIGTERM:
  gt_pose.csv     sim_t, x, y, z, yaw of the model (default industrial_amr),
                  whose frame is base_footprint. From the SceneBroadcaster's
                  /world/<world>/dynamic_pose/info, stamped with sim time.
  world_stats.csv wall_t, sim_t, real_t, real_time_factor from
                  /world/<world>/stats.

ros_gz_bridge's Pose_V -> TFMessage conversion drops the entity names and
stamps, so the poses are read here with names and sim stamps intact. Nothing
is published; the robot stack never sees these data.
"""

import argparse
import csv
import math
import signal
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node


def stamp_to_s(stamp):
    return stamp.sec + stamp.nsec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--world', default='industrial_room')
    parser.add_argument('--model', default='industrial_amr')
    parser.add_argument('--out-dir', required=True)
    args = parser.parse_args()
    lock = threading.Lock()
    pose_file = open(f'{args.out_dir}/gt_pose.csv', 'w', newline='')
    stats_file = open(f'{args.out_dir}/world_stats.csv', 'w', newline='')
    pose_csv = csv.writer(pose_file)
    stats_csv = csv.writer(stats_file)
    pose_csv.writerow(['sim_t', 'x', 'y', 'z', 'yaw'])
    stats_csv.writerow(['wall_t', 'sim_t', 'real_t', 'real_time_factor'])
    last_sim_t = [None]

    def on_poses(msg):
        t = stamp_to_s(msg.header.stamp)
        for p in msg.pose:
            if p.name == args.model:
                with lock:
                    if t != last_sim_t[0]:
                        last_sim_t[0] = t
                        pose_csv.writerow([f'{t:.4f}', f'{p.position.x:.5f}', f'{p.position.y:.5f}',
                                           f'{p.position.z:.5f}',
                                           f'{yaw_from_quaternion(p.orientation):.5f}'])
                break

    def on_stats(msg):
        with lock:
            stats_csv.writerow([f'{time.time():.4f}', f'{stamp_to_s(msg.sim_time):.4f}',
                                f'{stamp_to_s(msg.real_time):.4f}', f'{msg.real_time_factor:.4f}'])

    node = Node()
    ok_pose = node.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', on_poses)
    ok_stats = node.subscribe(WorldStatistics, f'/world/{args.world}/stats', on_stats)
    if not (ok_pose and ok_stats):
        raise SystemExit('gz-transport subscription failed')

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    while not stop.wait(1.0):
        with lock:
            pose_file.flush()
            stats_file.flush()
    with lock:
        pose_file.close()
        stats_file.close()


if __name__ == '__main__':
    main()
