#!/usr/bin/env python3
"""
ROS 2 Subscriber Node - Listener

Subscribes to the 'chatter' topic and logs received messages.
Part of the Physical AI & Humanoid Robotics book, Module 1.

Usage:
    ros2 run my_first_package listener
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """A simple subscriber node that receives messages from a topic."""

    def __init__(self):
        super().__init__('listener')

        # Create subscription: message type, topic name, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        self.get_logger().info('Listener node has started')

    def listener_callback(self, msg: String):
        """Called when a message is received on the topic."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Entry point for the listener node."""
    rclpy.init(args=args)

    node = ListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
