import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import subprocess
import re


class topicListPublisher(Node):

    def __init__(self):
        super().__init__('topics_publisher')
        
        # Publisher For All Topics in case other nodes want to access it?
        self.topic_publisher = self.create_publisher(String, 'topicList', 10)
        
        self.topic_list = []
        self.type_list = []
        self.get_topics()
        
        # Create Timer and Print
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        self.get_topics()
        
        # Publish Plain Topics
        concated = ''
        for s in self.topic_list:
            concated += s
        
        # add their types split by space
        concated += " "
        for s in self.type_list:
            concated += "/" + s
        
        msg = String()
        msg.data = concated
        
        
        self.topic_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {concated}")
    
    
    def get_topics(self) -> None:
        # Run Ros Topic List
        proc = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True)
        proc2 = subprocess.run(["ros2", "topic", "list", "-t"], capture_output=True, text=True)
        
        # Append All Topics
        for line in proc.stdout.split('\n'):
            if line in self.topic_list:
                continue
            self.topic_list.append(line)
        
        # Regex -t just trust 
        for line in proc2.stdout.split('\n'):
            black_magic = re.search("\[[a-zA-Z\/\_]*\]*", line)
            if black_magic == None:
                continue
            msgtype = black_magic.group()
            new = msgtype.replace('[', '')
            new = new.replace(']', '')
            tmp = re.search("[^/]+$", new)
            if tmp == None: continue
            new_s = tmp.group()
            if new_s in self.type_list:
                continue
            self.type_list.append(new_s)
        
        # Remove empty Doesnt work anymore 😢
        self.topic_list.remove('')
    
def main(args=None):
    rclpy.init(args=args)

    topic_publisher = topicListPublisher()

    rclpy.spin(topic_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()