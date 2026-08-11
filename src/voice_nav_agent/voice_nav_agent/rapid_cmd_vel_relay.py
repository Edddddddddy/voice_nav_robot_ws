"""Bypass the product safety chain in the explicitly unsafe rapid demo."""

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.node import Node


class RapidCmdVelRelay(Node):
    """Forward Nav2 controller output straight to the simulated base."""

    def __init__(self):
        super().__init__('rapid_cmd_vel_relay')
        self.received = 0
        self.nonzero_seen = False
        self.output = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10
        )
        self.input = self.create_subscription(
            TwistStamped, '/cmd_vel_nav', self._forward, 10
        )
        self.get_logger().warning(
            'Rapid demo bypass enabled: /cmd_vel_nav drives the base directly'
        )

    def _forward(self, message):
        self.received += 1
        self.output.publish(message)
        moving = abs(message.twist.linear.x) + abs(message.twist.angular.z)
        if moving > 1e-4 and not self.nonzero_seen:
            self.nonzero_seen = True
            self.get_logger().info(
                'First Nav2 velocity linear=%.3f angular=%.3f'
                % (message.twist.linear.x, message.twist.angular.z)
            )


def main():
    """Run the rapid-only velocity relay."""
    rclpy.init()
    node = RapidCmdVelRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
