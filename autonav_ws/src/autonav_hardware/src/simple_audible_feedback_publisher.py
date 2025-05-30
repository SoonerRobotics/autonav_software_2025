#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autonav_msgs.msg import AudibleFeedback
import os


class MinimalAudibleFeedbackPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(
            AudibleFeedback,
            '/autonav/audible_feedback',
            20
            )
        
        timer_period = 3 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        msg = AudibleFeedback()
        if self.i == 0:
            msg.filename = os.path.expanduser("~/Documents/vivalavida.wav")

        elif self.i % 3 == 0:
            msg.stop_all = True

        else:
            msg.filename = os.path.expanduser("~/Documents/metal-pipe.wav")

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.filename)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalAudibleFeedbackPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()