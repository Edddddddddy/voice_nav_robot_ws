"""Repeated initial-pose publisher for the fast Nav2 simulation path."""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class RapidInitialPose(Node):
    def __init__(self):
        super().__init__('rapid_initial_pose')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.remaining = 30
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685389
        self.publisher.publish(message)
        self.remaining -= 1
        if self.remaining == 0:
            self.get_logger().info('Initial pose published for Nav2 startup')
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
