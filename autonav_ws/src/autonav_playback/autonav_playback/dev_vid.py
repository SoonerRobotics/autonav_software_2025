import rclpy
from rclpy.node import Node

from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import *
from cv_bridge import CvBridge


from datetime import datetime
import os
import cv2

import subprocess

bridge = CvBridge()


class DevVidPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.vid_publisher = self.create_publisher(CompressedImage, 'dev_vid', 10)
        timer_period =  0.033 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # Open the video file
        self.video_path = '/root/robo/autonav_software_2025/autonav_ws/src/autonav_playback/autonav_playback/kitten.mp4'
        self.cap = cv2.VideoCapture(self.video_path)
        
        

    def timer_callback(self):
        
        if not self.cap.isOpened():
            print("Error: Could not open video")
            return
        
        ret, frame = self.cap.read()
        print('Frame has been read')
        if ret:
            image = bridge.cv2_to_compressed_imgmsg(frame)
            msg = image
        
        
        self.vid_publisher.publish(msg)
        self.get_logger().info('Publishing frame')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    dev_vid_publisher = DevVidPublisher()

    rclpy.spin(dev_vid_publisher)
    dev_vid_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()