"""
Frontier-based autonomous exploration on top of Nav2.

Decision layer only: it picks frontier goals from the SLAM map and sends them
to Nav2's NavigateToPose action.  Path planning, obstacle avoidance and the
collision monitor all stay in Nav2, so this node never publishes cmd_vel.

Loop (every ``update_period`` seconds):
  1. Detect and cluster frontiers in /map.
  2. Score clusters (bigger and closer is better), skipping blacklisted goals.
  3. Send the best goal.  While it is active, replace it (Nav2 preempts the
     running goal, so the robot does not stop) once the area around it is no
     longer unknown, i.e. the frontier was mapped before the robot arrived.
  4. Goals that Nav2 aborts or that time out are blacklisted.
When no frontiers remain, optionally return to the start pose and stop.
"""

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .frontier_detection import find_frontiers, GridInfo, UNKNOWN
from .pose_utils import make_pose, yaw_from_quaternion


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value
        self.update_period = self.declare_parameter('update_period', 1.0).value
        self.min_frontier_size = self.declare_parameter('min_frontier_size', 0.5).value
        self.obstacle_clearance = self.declare_parameter('obstacle_clearance', 0.35).value
        self.min_goal_distance = self.declare_parameter('min_goal_distance', 0.5).value
        self.size_weight = self.declare_parameter('size_weight', 1.0).value
        self.distance_weight = self.declare_parameter('distance_weight', 1.0).value
        self.blacklist_radius = self.declare_parameter('blacklist_radius', 0.5).value
        self.explored_radius = self.declare_parameter('explored_radius', 0.3).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 30.0).value
        self.return_to_start = self.declare_parameter('return_to_start', True).value
        # The SLAM map is briefly incomplete right after updates; only finish
        # after this many consecutive checks found no frontier.
        self.empty_checks_to_finish = self.declare_parameter('empty_checks_to_finish', 3).value

        self.grid = None
        self.grid_info = None
        self.start_pose = None
        self.blacklist = []
        self.goal_seq = 0          # identifies the current goal in callbacks
        self.goal_active = False
        self.goal_handle = None    # set once Nav2 accepts the goal
        self.goal_xy = None
        self.goal_sent_time = None
        self.goal_superseded = False   # current goal timed out or got explored
        self.empty_checks = 0
        self.finished = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(self.update_period, self.on_timer)

    def on_map(self, msg):
        self.grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        o = msg.info.origin
        self.grid_info = GridInfo(
            msg.info.resolution, o.position.x, o.position.y,
            yaw_from_quaternion(o.orientation))

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_from_quaternion(tr.rotation)

    def on_timer(self):
        if self.finished or self.grid is None:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().info(
                'Waiting for navigate_to_pose action server...', throttle_duration_sec=5.0)
            return
        pose = self.robot_pose()
        if pose is None:
            self.get_logger().info(
                'Waiting for map → robot transform...', throttle_duration_sec=5.0)
            return
        if self.start_pose is None:
            self.start_pose = pose

        if self.goal_active and not self.goal_should_be_replaced():
            return

        goal = self.select_frontier(pose)
        if goal is None:
            self.empty_checks += 1
            if self.empty_checks >= self.empty_checks_to_finish:
                self.finish(pose)
            return
        self.empty_checks = 0
        yaw = math.atan2(goal[1] - pose[1], goal[0] - pose[0])
        self.get_logger().info(f'Navigating to frontier ({goal[0]:.2f}, {goal[1]:.2f})')
        self.send_goal(goal[0], goal[1], yaw)

    def select_frontier(self, pose):
        """Return the best frontier goal (x, y), or None if none is left."""
        res = self.grid_info.resolution
        frontiers = find_frontiers(
            self.grid, self.grid_info,
            min_size_cells=max(1, round(self.min_frontier_size / res)),
            occupied_clearance_cells=self.obstacle_clearance / res)

        best, best_score = None, -math.inf
        for f in frontiers:
            goal = f.goal(pose[0], pose[1], self.min_goal_distance)
            if goal is None or self.is_blacklisted(*goal):
                continue
            dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
            score = self.size_weight * f.size * res - self.distance_weight * dist
            if score > best_score:
                best, best_score = goal, score
        self.get_logger().debug(f'{len(frontiers)} frontiers, best score {best_score:.2f}')
        return best

    def goal_should_be_replaced(self):
        if self.goal_handle is None:
            return False
        if self.goal_superseded:       # decided earlier, waiting for a new goal
            return True
        elapsed = self.get_clock().now() - self.goal_sent_time
        if elapsed > Duration(seconds=self.goal_timeout):
            self.get_logger().warn('Goal timed out, blacklisting it.')
            self.blacklist.append(self.goal_xy)
            self.goal_superseded = True
        elif not self.area_has_unknown(*self.goal_xy):
            self.get_logger().info('Frontier explored before arrival, moving on.')
            self.goal_superseded = True
        return self.goal_superseded

    def area_has_unknown(self, x, y):
        row, col = self.grid_info.world_to_cell(x, y)
        r = max(1, round(self.explored_radius / self.grid_info.resolution))
        h, w = self.grid.shape
        window = self.grid[max(0, row - r):min(h, row + r + 1),
                           max(0, col - r):min(w, col + r + 1)]
        return bool((window == UNKNOWN).any())

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius
                   for bx, by in self.blacklist)

    def finish(self, pose):
        self.finished = True
        sx, sy = self.start_pose[:2]
        dist = math.hypot(sx - pose[0], sy - pose[1])
        if not self.return_to_start or dist < self.min_goal_distance:
            self.get_logger().info('Exploration complete.')
            return
        # Arrive facing the direction of travel rather than the start heading:
        # rotating in place at the start can be blocked by nearby obstacles.
        self.get_logger().info('Exploration complete, returning to start position.')
        self.send_goal(sx, sy, math.atan2(sy - pose[1], sx - pose[0]))

    def send_goal(self, x, y, yaw):
        """Send a goal; an already running goal is preempted by Nav2."""
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw, self.global_frame, self.get_clock().now().to_msg())
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_xy = (x, y)
        self.goal_sent_time = self.get_clock().now()
        self.goal_active = True
        self.goal_handle = None
        self.goal_superseded = False
        self.nav_client.send_goal_async(goal).add_done_callback(
            lambda future: self.on_goal_response(future, seq))

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:   # superseded while waiting for acceptance
            return
        if not handle.accepted:
            # Rejection means Nav2 is not accepting goals (e.g. still being
            # activated), not that the frontier is unreachable: retry later.
            self.get_logger().warn('Goal rejected by Nav2, retrying.', throttle_duration_sec=5.0)
            self.goal_active = False
            self.finished = False   # a rejected return-to-start goal is resent too
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda future: self.on_goal_result(future, seq))

    def on_goal_result(self, future, seq):
        if seq != self.goal_seq:   # preempted goal: Nav2 reports it as aborted
            return
        status = future.result().status
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f'Nav2 aborted goal ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f}), '
                'blacklisting it.')
            self.blacklist.append(self.goal_xy)
        elif status == GoalStatus.STATUS_SUCCEEDED and self.finished:
            self.get_logger().info('Back at start. Save the map with map_saver_cli.')
        self.goal_active = False
        self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
