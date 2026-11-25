"""
Global localisation: recover the robot pose when it is unknown.

Scatters AMCL particles over the whole map, then rotates in place using Nav2's
Spin behaviour (collision-checked, sim-time aware) until the pose covariance
drops below ``max_xy_stddev`` or ``max_spins`` full turns have been made.
"""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty


class GlobalLocalizer(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='global_localizer')
        self.max_spins = self.declare_parameter('max_spins', 3).value
        self.max_xy_stddev = self.declare_parameter('max_xy_stddev', 0.25).value
        self.xy_stddev = math.inf
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl_pose, qos)
        self.reinit_client = self.create_client(Empty, 'reinitialize_global_localization')

    def on_amcl_pose(self, msg):
        cov = msg.pose.covariance
        self.xy_stddev = math.sqrt(max(cov[0], cov[7]))

    def run(self):
        # Not waitUntilNav2Active(): it would publish an initial pose itself.
        self._waitForNodeToActivate('amcl')
        self._waitForNodeToActivate('behavior_server')

        self.info('Scattering particles across the whole map...')
        self.reinit_client.wait_for_service()
        rclpy.spin_until_future_complete(self, self.reinit_client.call_async(Empty.Request()))
        self.xy_stddev = math.inf

        for turn in range(1, self.max_spins + 1):
            self.info(f'Spinning to converge particles (turn {turn}/{self.max_spins})...')
            self.spin(spin_dist=2.0 * math.pi, time_allowance=30)
            while not self.isTaskComplete():
                pass
            if self.getResult() != TaskResult.SUCCEEDED:
                self.error(f'Spin failed: {self.getResult().name}')
                return False
            rclpy.spin_once(self, timeout_sec=0.5)  # collect the latest amcl_pose
            self.info(f'Position std-dev: {self.xy_stddev:.3f} m')
            if self.xy_stddev <= self.max_xy_stddev:
                self.info('Localisation converged.')
                return True

        self.warn('Localisation did not converge; set the pose manually in RViz.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizer()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
