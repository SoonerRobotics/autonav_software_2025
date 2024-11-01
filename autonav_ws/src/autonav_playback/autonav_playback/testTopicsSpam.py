### Flood Ros with topics in order to test topic_pub node
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from autonav_msgs.msg import SystemState
from autonav_msgs.srv import SetSystemState

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetSystemState, 'set_system_state', self.set_systemstate_callback)

    def set_systemstate_callback(self, request, response):
        response.ok = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


class SpamPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic1', 10)
        self.publisher_2 = self.create_publisher(String, 'topic2', 10)
        self.publisher_3 = self.create_publisher(String, 'topic3', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.cli = self.create_client(SetSystemState, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetSystemState.Request()
        
    def send_request(self, a, b):
        self.req.state = a
        self.req.mobility = b
        return self.cli.call_async(self.req)
        

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.publisher_2.publish(msg)
        self.publisher_3.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    spam_publisher = SpamPublisher()
    future = spam_publisher.send_request(int(sys.argv[1]), int(sys.argv[False]))
    rclpy.spin_until_future_complete(spam_publisher, future)
    
    response = future.result()
    spam_publisher.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[False]), response.ok))

    spam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()