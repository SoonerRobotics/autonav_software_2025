import rclpy
from rclpy.node import Node

from std_msgs.msg import ByteMultiArray

import subprocess

class TopicPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(ByteMultiArray, 'topicList', 10)
        
        self.topic_list = []
        self.get_topics()
        
        # Purge unessecary topics
        
        
        # Create Timer and Print
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        self.get_topics()
        
        # Convert / Ecode Strings
        ba = bytearray()
        for s in self.topic_list:
            ba.extend(s.encode('utf-8'))
        
        print(self.topic_list)
        self.get_logger().info(f"Publishing: {ba}")
    
    
    def get_topics(self) -> None:
        # Run Ros Topic List
        proc = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True)
        
        # Append All Topics
        for line in proc.stdout.split('\n'):
            if line in self.topic_list:
                continue
            self.topic_list.append(line)
        
        # Remove empty
        self.topic_list.remove('')
        
        # Remove '/'
        #for i in range(len(self.topic_list)):
            #self.topic_list[i] = self.topic_list[i].replace('/', '')
    
    def strToByte(self, string: str) -> bytes:
        output = string.encode("utf-8")
        return output
        


def main(args=None):
    rclpy.init(args=args)

    topic_publisher = TopicPublisher()

    rclpy.spin(topic_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()