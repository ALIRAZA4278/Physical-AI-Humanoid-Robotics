#!/usr/bin/env python3
"""
ROS 2 Publisher Node - Talker

Publishes string messages to the 'chatter' topic at 2 Hz.
Part of the Physical AI & Humanoid Robotics book, Module 1.

Usage:
    ros2 run my_first_package talker
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """A simple publisher node that sends messages to a topic."""

    def __init__(self):
        super().__init__('talker')

        # Create publisher: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer: period in seconds, callback function
        timer_period = 0.5  # 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.count = 0
        self.get_logger().info('Talker node has started')

    def timer_callback(self):
        """Called every timer_period seconds."""
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    """Entry point for the talker node."""
    rclpy.init(args=args)

    node = TalkerNode()

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
