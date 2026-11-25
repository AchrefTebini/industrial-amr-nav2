#!/usr/bin/env python3
"""
Run the simulation experiments headless and record raw data.

Usage (from the workspace root, ROS and the workspace sourced):
  python3 paper/experiments/run_experiments.py [options] pilot
  python3 paper/experiments/run_experiments.py [options] mapping --runs 10
  python3 paper/experiments/run_experiments.py [options] navigation --sessions 3 --loops 3
  python3 paper/experiments/run_experiments.py [options] spin --sessions 3
mapping, navigation and spin also take --first N to start numbering at N.

Options (before the mode):
  --data-dir DIR       where runs go (default paper/data)
  --condition LABEL    label stored in each run's provenance.json
  --expect-axle-x X    refuse to run unless the installed URDF puts the drive-axle
                       midpoint at x = X in base_footprint (guards against
                       recording one condition under another's label)
  --prereg FILE        protocol file whose sha256 goes into provenance.json

Each run directory holds a rosbag (mcap), the Gazebo ground truth (gt_logger.py),
the live event log, process logs, the saved map (mapping runs only), and snapshots
of what was launched: robot_description.urdf (the installed xacro, rendered as
rsp.launch.py does), nav2_params.yaml, waypoints.yaml and provenance.json (git
state, package versions, file hashes). Outcomes are appended to <data dir>/runs.csv.
Spin mode runs spin_test.py on navigation.launch.py.

Before each run, stale processes are stopped by PID (never pkill -f), and the ROS
daemon and FastDDS shared memory are reset. Each child runs in its own session
with SIGINT restored to the default, because a shell started with `&` ignores
SIGINT and ros2 launch could not forward it. Launches are stopped with SIGINT,
and whatever survives is swept by session id.

An infrastructure failure (Nav2 not active within 120 s, or a stalled simulation
clock) is logged, its directory is renamed infra_*, and the run is retried; it is
never counted as a robot result.
"""

import argparse
import csv
import datetime
import hashlib
import json
import os
import platform
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET

from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node as GzNode
from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import world_geometry as wg

WS = Path(__file__).resolve().parents[2]
DATA = WS / 'paper' / 'data'
EXPERIMENTS = WS / 'paper' / 'experiments'

READY_TIMEOUT_WALL = 120.0
MAPPING_CAP_SIM = 600.0
NAV_CAP_SIM_PER_LOOP = 300.0
LOCALIZER_CAP_SIM = 150.0
SPIN_CAP_SIM = 180.0

BAG_TOPICS = [
    '/clock', '/map', '/tf', '/tf_static', '/rosout', '/odom', '/amcl_pose',
    '/navigate_to_pose/_action/status', '/navigate_to_pose/_action/feedback',
    '/collision_monitor_state',
]


STALE_PATTERNS = [
    'gzserver', 'gz sim', 'parameter_bridge', 'image_bridge', 'robot_state_publisher',
    'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
    'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor',
    'route_server', 'docking_server', 'lifecycle_manager', 'map_server',
    'costmap_filter_info_server', 'amcl', 'slam_toolbox', 'frontier_explorer',
    'waypoint_runner', 'global_localizer', 'ros2 bag record', 'gt_logger.py',
    'ros2 launch industrial_amr', 'map_saver_cli', 'ros_gz_sim/create', 'spin_test.py',
]

# Versions recorded in provenance.json.
PACKAGES = [
    'ros-jazzy-nav2-bringup', 'ros-jazzy-nav2-mppi-controller', 'ros-jazzy-nav2-costmap-2d',
    'ros-jazzy-nav2-amcl', 'ros-jazzy-slam-toolbox', 'ros-jazzy-ros-gz-sim', 'ros-jazzy-rclpy',
    'ros-jazzy-rosbag2', 'libgz-sim8',
]


def log(msg):
    stamp = datetime.datetime.now().strftime('%H:%M:%S')
    print(f'[{stamp}] {msg}', flush=True)


def _child_setup():
    os.setsid()
    signal.signal(signal.SIGINT, signal.SIG_DFL)


def start(cmd, log_path):
    out = open(log_path, 'w')
    proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, cwd=WS,
                            preexec_fn=_child_setup)
    proc.log_file = out
    return proc


def session_pids(sid):
    res = subprocess.run(['pgrep', '-s', str(sid)], capture_output=True, text=True)
    return [int(p) for p in res.stdout.split()]


def stop(proc, name, timeout=30.0):
    """SIGINT the process, wait, then sweep its whole session by PID."""
    if proc is None:
        return
    sid = proc.pid
    if proc.poll() is None:
        os.kill(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            log(f'{name}: no exit {timeout:.0f} s after SIGINT')
    for sig, wait in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        pids = session_pids(sid)
        if not pids:
            break
        log(f'{name}: sending {sig.name} to leftover PIDs {pids}')
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(wait)
    try:
        proc.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        pass
    proc.log_file.close()


NEVER_KILL = {'bash', 'sh', 'dash', 'zsh', 'fish', 'sudo', 'timeout', 'grep', 'tail',
              'less', 'cat', 'sed', 'awk', 'pgrep', 'ps', 'watch', 'nano', 'vim', 'code',
              'node'}


def stale_pids():
    """Robot processes left over from earlier runs, matched on what they execute."""
    me = {os.getpid(), os.getppid()}
    found = {}
    for pattern in STALE_PATTERNS:
        res = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True)
        for pid in map(int, res.stdout.split()):
            if pid in me:
                continue
            try:
                argv = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
            except OSError:
                continue
            argv = [a.decode(errors='replace') for a in argv if a]
            if not argv or Path(argv[0]).name in NEVER_KILL:
                continue

            exe = Path(argv[0]).name
            interpreted = exe.startswith('python') or exe in ('ruby', 'gz')
            target = ' '.join(argv[:4]) if interpreted else argv[0]

            if not re.search(rf'(?<![A-Za-z0-9]){re.escape(pattern)}(?![A-Za-z0-9])', target):
                continue
            cmd = ' '.join(argv)
            if 'run_experiments.py' in cmd:
                continue
            found[pid] = cmd
    return found


def hygiene():
    for sig in (signal.SIGTERM, signal.SIGKILL):
        pids = stale_pids()
        if not pids:
            break
        for pid, cmd in pids.items():
            log(f'stale process {pid}: {cmd[:100]} -> {sig.name}')
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                log(f'stale process {pid}: not ours, left alone')
        time.sleep(3.0)
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)
    subprocess.run(['fastdds', 'shm', 'clean'], capture_output=True)
    subprocess.run(['ros2', 'daemon', 'start'], capture_output=True)


def installed_share(pkg):
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', pkg], capture_output=True, text=True,
                            check=True).stdout.strip()
    return Path(prefix) / 'share' / pkg


def render_urdf():
    """The installed xacro rendered as rsp.launch.py renders it for the simulation."""
    xacro_file = (installed_share('industrial_amr_description') / 'urdf'
                  / 'industrial_amr.urdf.xacro')
    return subprocess.run(['xacro', str(xacro_file), 'use_sim:=true'], capture_output=True,
                          text=True, check=True).stdout


def axle_x(urdf):
    """x of the drive-axle midpoint in base_footprint, from a rendered URDF.

    base_footprint -> base_link -> {left,right}_wheel_joint; both wheel joints
    must agree.
    """
    root = ET.fromstring(urdf)

    def joint_x(name):
        joint = root.find(f"joint[@name='{name}']")
        return float(joint.find('origin').get('xyz').split()[0])

    left, right = joint_x('left_wheel_joint'), joint_x('right_wheel_joint')
    if left != right:
        raise SystemExit(f'wheel joints disagree in x: {left} vs {right}')
    return joint_x('base_footprint_joint') + left


def check_axle(expected):
    """Stop unless the installed URDF has the drive axle where `expected` says."""
    if expected is None:
        return
    actual = axle_x(render_urdf())
    if abs(actual - expected) > 1e-9:
        raise SystemExit(f'installed URDF has the drive axle at x = {actual} in base_footprint, '
                         f'expected {expected}; wrong branch or build?')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.run(['git', *args], cwd=WS, capture_output=True,
                          text=True).stdout.rstrip('\n')


def static_provenance(args):
    """Facts that don't change during one invocation."""
    versions = subprocess.run(['dpkg-query', '-W', '-f=${Package} ${Version}\\n', *PACKAGES],
                              capture_output=True, text=True).stdout.split('\n')
    return {
        'condition': args.condition,
        'expect_axle_x': args.expect_axle_x,
        'prereg': ({'path': str(Path(args.prereg).resolve()), 'sha256': sha256(args.prereg)}
                   if args.prereg else None),
        'command': sys.argv,
        'kernel': platform.release(),
        'packages': dict(v.split(' ', 1) for v in versions if v),
        'tooling_sha256': {p.name: sha256(p) for p in sorted(EXPERIMENTS.glob('*.py'))},
    }


def snapshot(run_dir, static):
    """Copy what this run launches into run_dir and write provenance.json."""
    urdf = render_urdf()
    (run_dir / 'robot_description.urdf').write_text(urdf)
    shutil.copyfile(installed_share('industrial_amr_navigation') / 'config' / 'nav2_params.yaml',
                    run_dir / 'nav2_params.yaml')
    shutil.copyfile(installed_share('industrial_amr_exploration') / 'config' / 'waypoints.yaml',
                    run_dir / 'waypoints.yaml')
    names = ('robot_description.urdf', 'nav2_params.yaml', 'waypoints.yaml')
    prov = {
        **static,
        'run': run_dir.name,
        'wall_start': datetime.datetime.now().isoformat(timespec='seconds'),
        'git_commit': git('rev-parse', 'HEAD'),
        'git_branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
        'git_dirty': git('status', '--porcelain', '--untracked-files=no').splitlines(),
        'axle_x': axle_x(urdf),
        'footprint_bounds': wg.footprint_bounds(run_dir / 'nav2_params.yaml'),
        'snapshot_sha256': {n: sha256(run_dir / n) for n in names},
    }
    (run_dir / 'provenance.json').write_text(json.dumps(prov, indent=2) + '\n')


class Monitor(Node):
    """Collects /rosout events; sim time comes from Gazebo's world stats."""

    def __init__(self):
        super().__init__('paper_eval_monitor')
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, '/rosout', self.on_log, qos)
        self.lock = threading.Lock()
        self.events = []
        self.sim_time = None
        self.gz = GzNode()
        self.gz.subscribe(WorldStatistics, '/world/industrial_room/stats', self.on_stats)

    def on_stats(self, msg):
        self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def on_log(self, msg):
        with self.lock:
            self.events.append((time.time(), self.sim_time, msg.name, msg.level, msg.msg))

    def reset(self):
        with self.lock:
            self.events = []
        self.sim_time = None

    def find(self, node_substr, text, since=0):
        with self.lock:
            for i, (_, _, name, _, msg) in enumerate(self.events[since:], start=since):
                if node_substr in name and text in msg:
                    return i
        return None

    def dump(self, path):
        with self.lock, open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['wall_t', 'sim_t', 'node', 'level', 'msg'])
            for wall_t, sim_t, name, level, msg in self.events:
                w.writerow([f'{wall_t:.3f}', '' if sim_t is None else f'{sim_t:.3f}',
                            name, level, msg])


def wait_until(predicate, wall_timeout=None, sim_timeout=None, monitor=None, poll=0.5):
    """Wait for predicate(); return 'ok', 'wall_timeout' or 'sim_timeout'."""
    t0 = time.time()
    sim0 = monitor.sim_time if monitor else None
    while True:
        if predicate():
            return 'ok'
        if wall_timeout is not None and time.time() - t0 > wall_timeout:
            return 'wall_timeout'
        if sim_timeout is not None and monitor.sim_time is not None:
            if sim0 is None:
                sim0 = monitor.sim_time
            elif monitor.sim_time - sim0 > sim_timeout:
                return 'sim_timeout'
        time.sleep(poll)


def append_run(data_dir, row):
    path = data_dir / 'runs.csv'
    new = not path.exists()
    with open(path, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['run', 'mode', 'counted', 'wall_start', 'wall_end', 'outcome',
                        'sim_end', 'notes'])
        w.writerow(row)


def run_once(mode, run_dir, monitor, static, loops=3):
    """One mapping run, navigation session or spin test. Returns (outcome, notes)."""
    run_dir.mkdir(parents=True)
    snapshot(run_dir, static)
    monitor.reset()
    procs = {}
    notes = []
    try:
        procs['gt'] = start([sys.executable, str(EXPERIMENTS / 'gt_logger.py'),
                             '--out-dir', str(run_dir)], run_dir / 'gt_logger.log')
        procs['bag'] = start(['ros2', 'bag', 'record', '-s', 'mcap', '--include-hidden-topics',
                              '-o', str(run_dir / 'bag'), *BAG_TOPICS], run_dir / 'bag.log')
        launch_file = 'mapping.launch.py' if mode == 'mapping' else 'navigation.launch.py'
        procs['launch'] = start(['ros2', 'launch', 'industrial_amr_bringup', launch_file,
                                 'headless:=true', 'rviz:=false'], run_dir / 'launch.log')

        managers = ['lifecycle_manager_navigation']
        if mode in ('navigation', 'spin'):
            managers.append('lifecycle_manager_localization')
        ready = wait_until(
            lambda: all(monitor.find(m, 'Managed nodes are active') is not None
                        for m in managers),
            wall_timeout=READY_TIMEOUT_WALL)
        if ready != 'ok':
            return 'infra', f'Nav2 not active within {READY_TIMEOUT_WALL:.0f} s wall'
        log(f'{run_dir.name}: Nav2 active at sim {monitor.sim_time}')

        if mode == 'mapping':
            res = wait_until(
                lambda: (monitor.find('frontier_explorer', 'Back at start') is not None
                         or monitor.find('frontier_explorer', 'Exploration complete.') is not None),
                sim_timeout=MAPPING_CAP_SIM, wall_timeout=3 * MAPPING_CAP_SIM, monitor=monitor)
            if res == 'wall_timeout':
                return 'infra', 'simulation clock stalled'
            outcome = 'completed' if res == 'ok' else 'cap'
            time.sleep(3.0)
            saver = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(run_dir / 'map'),
                 '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=60, cwd=WS)
            (run_dir / 'map_saver.log').write_text(saver.stdout + saver.stderr)
            if not (run_dir / 'map.pgm').exists():
                notes.append('map_saver_cli failed')
            return outcome, '; '.join(notes)

        if mode == 'spin':
            procs['runner'] = start([sys.executable, str(EXPERIMENTS / 'spin_test.py'),
                                     '--ros-args', '-p', 'use_sim_time:=true'],
                                    run_dir / 'spin_test.log')
            res = wait_until(lambda: procs['runner'].poll() is not None,
                             sim_timeout=SPIN_CAP_SIM, wall_timeout=3 * SPIN_CAP_SIM,
                             monitor=monitor)
            if res == 'wall_timeout':
                return 'infra', 'simulation clock stalled'
            if res != 'ok':
                return 'cap', '; '.join(notes)
            code = procs['runner'].returncode
            return ('completed' if code == 0 else 'failed'), f'spin_test exit code {code}'

        share = subprocess.run(['ros2', 'pkg', 'prefix', 'industrial_amr_exploration'],
                               capture_output=True, text=True).stdout.strip()
        params = f'{share}/share/industrial_amr_exploration/config/waypoints.yaml'
        procs['runner'] = start(['ros2', 'run', 'industrial_amr_exploration', 'waypoint_runner',
                                 '--ros-args', '--params-file', params, '-p', f'loops:={loops}'],
                                run_dir / 'waypoint_runner.log')
        res = wait_until(lambda: procs['runner'].poll() is not None,
                         sim_timeout=NAV_CAP_SIM_PER_LOOP * loops,
                         wall_timeout=3 * NAV_CAP_SIM_PER_LOOP * loops, monitor=monitor)
        if res == 'wall_timeout':
            return 'infra', 'simulation clock stalled'
        outcome = 'completed' if res == 'ok' else 'cap'
        stop(procs.pop('runner'), 'waypoint_runner', timeout=10)
        if outcome == 'completed':
            procs['localizer'] = start(['ros2', 'run', 'industrial_amr_exploration',
                                        'global_localizer', '--ros-args', '-p',
                                        'use_sim_time:=true'],
                                       run_dir / 'global_localizer.log')
            res = wait_until(lambda: procs['localizer'].poll() is not None,
                             sim_timeout=LOCALIZER_CAP_SIM, wall_timeout=3 * LOCALIZER_CAP_SIM,
                             monitor=monitor)
            if res != 'ok':
                notes.append(f'global_localizer {res}')
            stop(procs.pop('localizer'), 'global_localizer', timeout=10)
        return outcome, '; '.join(notes)
    finally:
        monitor.dump(run_dir / 'events_live.csv')
        for name in ('runner', 'localizer', 'bag', 'gt', 'launch'):
            if name in procs:
                stop(procs.pop(name), name, timeout=30 if name == 'launch' else 10)


def run_series(mode, base_dir, ids, monitor, ctx, loops=3, counted=True, max_retries=3):
    """ctx: data_dir, expect_axle_x and the static provenance of this invocation."""
    for run_id in ids:
        for attempt in range(1, max_retries + 1):
            run_dir = base_dir / run_id
            if run_dir.exists():
                raise SystemExit(f'{run_dir} exists; refusing to overwrite')
            check_axle(ctx['expect_axle_x'])
            hygiene()
            wall_start = datetime.datetime.now().isoformat(timespec='seconds')
            log(f'{mode} {run_id}: start (attempt {attempt})')
            outcome, notes = run_once(mode, run_dir, monitor, ctx['static'], loops)
            sim_end = '' if monitor.sim_time is None else f'{monitor.sim_time:.1f}'
            wall_end = datetime.datetime.now().isoformat(timespec='seconds')
            log(f'{mode} {run_id}: {outcome} {notes}')
            if outcome == 'infra':
                stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
                infra_dir = base_dir / f'infra_{run_id}_{stamp}'
                shutil.move(str(run_dir), str(infra_dir))
                append_run(ctx['data_dir'], [infra_dir.name, mode, 'no', wall_start, wall_end,
                                             outcome, sim_end, notes])
                continue
            append_run(ctx['data_dir'], [f'{base_dir.name}/{run_id}', mode,
                                         'yes' if counted else 'pilot', wall_start, wall_end,
                                         outcome, sim_end, notes])
            break
        else:
            log(f'{mode} {run_id}: giving up after {max_retries} infrastructure failures')
    hygiene()


def main():
    parser = argparse.ArgumentParser(description='Run the paper experiments.')
    parser.add_argument('--data-dir', default=str(DATA))
    parser.add_argument('--condition', default='')
    parser.add_argument('--expect-axle-x', type=float, default=None)
    parser.add_argument('--prereg', default=None)
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('pilot')
    p_map = sub.add_parser('mapping')
    p_map.add_argument('--runs', type=int, default=10)
    p_map.add_argument('--first', type=int, default=1)
    p_nav = sub.add_parser('navigation')
    p_nav.add_argument('--sessions', type=int, default=3)
    p_nav.add_argument('--first', type=int, default=1)
    p_nav.add_argument('--loops', type=int, default=3)
    p_spin = sub.add_parser('spin')
    p_spin.add_argument('--sessions', type=int, default=3)
    p_spin.add_argument('--first', type=int, default=1)
    args = parser.parse_args()

    data_dir = Path(args.data_dir).resolve()
    data_dir.mkdir(parents=True, exist_ok=True)
    check_axle(args.expect_axle_x)
    ctx = {'data_dir': data_dir, 'expect_axle_x': args.expect_axle_x,
           'static': static_provenance(args)}

    rclpy.init()
    monitor = Monitor()
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    threading.Thread(target=executor.spin, daemon=True).start()
    code = 0
    try:
        if args.cmd == 'pilot':
            run_series('mapping', data_dir / 'pilot', ['mapping'], monitor, ctx, counted=False)
            run_series('navigation', data_dir / 'pilot', ['navigation'], monitor, ctx, loops=1,
                       counted=False)
            run_series('spin', data_dir / 'pilot', ['spin'], monitor, ctx, counted=False)
        elif args.cmd == 'mapping':
            ids = [f'run_{i:02d}' for i in range(args.first, args.first + args.runs)]
            run_series('mapping', data_dir / 'mapping', ids, monitor, ctx)
        elif args.cmd == 'navigation':
            ids = [f'session_{i}' for i in range(args.first, args.first + args.sessions)]
            run_series('navigation', data_dir / 'navigation', ids, monitor, ctx, loops=args.loops)
        else:
            ids = [f'session_{i}' for i in range(args.first, args.first + args.sessions)]
            run_series('spin', data_dir / 'spin', ids, monitor, ctx)
    except BaseException as e:
        code = e.code if isinstance(e, SystemExit) and isinstance(e.code, int) else 1
        if isinstance(e, SystemExit) and not isinstance(e.code, int):
            print(e.code, file=sys.stderr)
        elif not isinstance(e, SystemExit):
            traceback.print_exc()
        log(f'stopped by {type(e).__name__}; exit code {code}')
    finally:
        executor.shutdown()
        monitor.destroy_node()
        rclpy.try_shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)


if __name__ == '__main__':
    main()
