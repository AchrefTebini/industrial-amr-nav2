#!/usr/bin/env python3
"""
Extract a run's rosbag into CSV/NPZ files next to it.

Usage: python3 paper/experiments/extract_bag.py <run_dir> [<run_dir> ...]

Bag receive times are wall clock; /clock in the same bag maps them to
simulation time, which every output uses (column sim_t).

Outputs in <run_dir>:
  clock.csv        wall_t, sim_t (thinned to 10 ms of sim time)
  est_pose.csv     sim_t, x, y, yaw: map->base_footprint as the robot's own
                   estimate (map->odom from slam_toolbox or AMCL composed with
                   odom->base_footprint from the drive plugin)
  amcl_pose.csv    sim_t (header), x, y, yaw, var_x, var_y, var_yaw
  odom.csv         sim_t (header), x, y, yaw: odom->base_footprint from the drive
                   plugin's /odom, every message (50 Hz)
  nav_status.csv   goal, status, accept_sim_t, sim_t: one row per new status
  nav_feedback.csv goal, sim_t, number_of_recoveries (rows where it changes)
  cm_state.csv     sim_t, action_type, polygon
  events.csv       sim_t, node, level, msg (/rosout)
  maps.npz         t, width, height, resolution, origin_x, origin_y, grid_<i>
"""

import csv
import math
from pathlib import Path
import sys
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Planar pose composition a * b, poses as (x, y, yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, ath + bth


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, deserialize_message(data, types[topic]), t * 1e-9


def extract(run_dir):
    run_dir = Path(run_dir)
    msgs = list(read_bag(run_dir / 'bag'))

    # Wall -> sim mapping from /clock.
    clock_wall, clock_sim = [], []
    for topic, msg, wall in msgs:
        if topic == '/clock':
            clock_wall.append(wall)
            clock_sim.append(stamp_s(msg.clock))
    clock_wall, clock_sim = np.array(clock_wall), np.array(clock_sim)
    order = np.argsort(clock_wall)
    clock_wall, clock_sim = clock_wall[order], clock_sim[order]

    def to_sim(wall):
        return float(np.interp(wall, clock_wall, clock_sim))

    with open(run_dir / 'clock.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['wall_t', 'sim_t'])
        last = -1.0
        for wt, st in zip(clock_wall, clock_sim):
            if st - last >= 0.01:
                w.writerow([f'{wt:.4f}', f'{st:.4f}'])
                last = st

    est_rows, amcl_rows, status_rows, fb_rows, cm_rows, ev_rows = [], [], [], [], [], []
    odom_rows = []
    maps = []
    map_odom = None
    last_est_t = -1.0
    seen_status = set()
    last_recoveries = {}

    for topic, msg, wall in msgs:
        if topic == '/tf':
            for tf in msg.transforms:
                tr = tf.transform
                pose = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
                if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                    map_odom = pose
                elif (tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint'
                      and map_odom is not None):
                    t = stamp_s(tf.header.stamp)
                    if t - last_est_t >= 0.1:
                        x, y, th = compose(map_odom, pose)
                        est_rows.append([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}',
                                         f'{math.atan2(math.sin(th), math.cos(th)):.4f}'])
                        last_est_t = t
        elif topic == '/amcl_pose':
            p, cov = msg.pose.pose, msg.pose.covariance
            amcl_rows.append([f'{stamp_s(msg.header.stamp):.3f}', f'{p.position.x:.4f}',
                              f'{p.position.y:.4f}', f'{yaw_of(p.orientation):.4f}',
                              f'{cov[0]:.6f}', f'{cov[7]:.6f}', f'{cov[35]:.6f}'])
        elif topic == '/odom':
            p = msg.pose.pose
            odom_rows.append([f'{stamp_s(msg.header.stamp):.3f}', f'{p.position.x:.5f}',
                              f'{p.position.y:.5f}', f'{yaw_of(p.orientation):.5f}'])
        elif topic == '/navigate_to_pose/_action/status':
            for st in msg.status_list:
                goal = bytes(st.goal_info.goal_id.uuid).hex()
                if (goal, st.status) not in seen_status:
                    seen_status.add((goal, st.status))
                    status_rows.append([goal, st.status, f'{stamp_s(st.goal_info.stamp):.3f}',
                                        f'{to_sim(wall):.3f}'])
        elif topic == '/navigate_to_pose/_action/feedback':
            goal = bytes(msg.goal_id.uuid).hex()
            n = msg.feedback.number_of_recoveries
            if last_recoveries.get(goal) != n:
                last_recoveries[goal] = n
                fb_rows.append([goal, f'{to_sim(wall):.3f}', n])
        elif topic == '/collision_monitor_state':
            cm_rows.append([f'{to_sim(wall):.3f}', msg.action_type, msg.polygon_name])
        elif topic == '/rosout':
            ev_rows.append([f'{to_sim(wall):.3f}', msg.name, msg.level, msg.msg])
        elif topic == '/map':
            info = msg.info
            maps.append((stamp_s(msg.header.stamp), info.width, info.height, info.resolution,
                         info.origin.position.x, info.origin.position.y,
                         np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)))

    def write(name, header, rows):
        with open(run_dir / name, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)

    write('est_pose.csv', ['sim_t', 'x', 'y', 'yaw'], est_rows)
    write('amcl_pose.csv', ['sim_t', 'x', 'y', 'yaw', 'var_x', 'var_y', 'var_yaw'], amcl_rows)
    write('odom.csv', ['sim_t', 'x', 'y', 'yaw'], odom_rows)
    write('nav_status.csv', ['goal', 'status', 'accept_sim_t', 'sim_t'], status_rows)
    write('nav_feedback.csv', ['goal', 'sim_t', 'number_of_recoveries'], fb_rows)
    write('cm_state.csv', ['sim_t', 'action_type', 'polygon'], cm_rows)
    write('events.csv', ['sim_t', 'node', 'level', 'msg'], ev_rows)
    if maps:
        np.savez_compressed(
            run_dir / 'maps.npz',
            t=np.array([m[0] for m in maps]), width=np.array([m[1] for m in maps]),
            height=np.array([m[2] for m in maps]), resolution=np.array([m[3] for m in maps]),
            origin_x=np.array([m[4] for m in maps]), origin_y=np.array([m[5] for m in maps]),
            **{f'grid_{i:04d}': m[6] for i, m in enumerate(maps)})
    print(f'{run_dir}: {len(msgs)} msgs, {len(est_rows)} est poses, {len(amcl_rows)} amcl, '
          f'{len(odom_rows)} odom, '
          f'{len(status_rows)} status rows, {len(cm_rows)} cm states, {len(ev_rows)} events, '
          f'{len(maps)} maps')


if __name__ == '__main__':
    for d in sys.argv[1:]:
        extract(d)
