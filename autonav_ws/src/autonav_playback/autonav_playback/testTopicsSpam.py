### Flood Ros with topics in order to test topic_pub node

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SpamPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic1', 10)
        self.publisher_2 = self.create_publisher(String, 'topic2', 10)
        self.publisher_3 = self.create_publisher(String, 'topic3', 10)
        self.publisher_4 = self.create_publisher(String, 'topic4', 10)
        self.publisher_5 = self.create_publisher(String, 'topic5', 10)
        self.publisher_6 = self.create_publisher(String, 'topic6', 10)
        self.publisher_7 = self.create_publisher(String, 'topic7', 10)
        self.publisher_8 = self.create_publisher(String, 'topic8', 10)
        self.publisher_9 = self.create_publisher(String, 'topic9', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.publisher_2.publish(msg)
        self.publisher_3.publish(msg)
        self.publisher_4.publish(msg)
        self.publisher_5.publish(msg)
        self.publisher_6.publish(msg)
        self.publisher_7.publish(msg)
        self.publisher_8.publish(msg)
        self.publisher_9.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    spam_publisher = SpamPublisher()

    rclpy.spin(spam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()