"""Repeated initial-pose publisher for the fast Nav2 simulation path."""

from geometry_msgs.msg import PoseWithCovarianceStamped

import rclpy
from rclpy.node import Node


class RapidInitialPose(Node):
    def __init__(self):
        super().__init__('rapid_initial_pose')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.remaining = 3
        self.timer = self.create_timer(0.5, self.publish)

    def publish(self):
        if self.publisher.get_subscription_count() == 0:
            return
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685389
        self.publisher.publish(message)
        self.remaining -= 1
        if self.remaining == 0:
            self.get_logger().info('Initial pose burst published for Nav2 startup')
            self.timer.cancel()


def main():
    rclpy.init()
    node = RapidInitialPose()
    try:
        rclpy.spin(node)
    finally:
        if node.context.ok():
            node.destroy_node()
        rclpy.shutdown()
