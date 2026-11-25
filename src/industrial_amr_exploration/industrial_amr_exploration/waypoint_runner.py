"""
Drive a waypoint circuit through Nav2 and report the success rate.

Waypoints come from the ``waypoints`` parameter as a flat list
[x0, y0, yaw0, x1, y1, yaw1, ...] in ``frame_id`` (see config/waypoints.yaml).
"""

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

from .pose_utils import make_pose


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator(node_name='waypoint_runner')
    flat = nav.declare_parameter('waypoints', [0.0, 0.0, 0.0]).value
    frame_id = nav.declare_parameter('frame_id', 'map').value
    loops = nav.declare_parameter('loops', 1).value

    if len(flat) % 3 != 0:
        nav.error('waypoints must be a flat list of (x, y, yaw) triples')
        rclpy.shutdown()
        return
    triples = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

    # Not waitUntilNav2Active(): with AMCL it publishes an initial pose at the
    # origin, which would overwrite the current localisation estimate.
    nav._waitForNodeToActivate('bt_navigator')

    succeeded = 0
    total = loops * len(triples)
    for loop in range(loops):
        for i, (x, y, yaw) in enumerate(triples, start=1):
            nav.info(f'[loop {loop + 1}/{loops}] waypoint {i}/{len(triples)}: '
                     f'({x:.2f}, {y:.2f}, {yaw:.2f})')
            nav.goToPose(make_pose(x, y, yaw, frame_id, nav.get_clock().now().to_msg()))
            while not nav.isTaskComplete():
                pass
            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                succeeded += 1
            else:
                nav.warn(f'Waypoint {i} finished with {result.name}')

    nav.info(f'Done: {succeeded}/{total} waypoints reached.')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
