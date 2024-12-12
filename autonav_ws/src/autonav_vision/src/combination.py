#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

import rclpy.qos
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState



bridge = CvBridge()


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_vision_combiner")

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        self.image_front = None
        self.image_left = None
        self.image_right = None
        self.image_back = None

        self.image_front_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/front", self.image_received_front, 1)
        self.image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/left", self.image_received_left, 1)
        self.image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/right", self.image_received_right, 1)
        self.image_back_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/back", self.image_received_back, 1)

        self.combined_image_publisher = self.create_publisher(CompressedImage, "/autonav/vision/combined/filtered", 1)

        self.set_device_state(DeviceState.READY)

    def image_received_front(self, msg):
        self.image_front = msg
        self.try_combine_images()

    def image_received_left(self, msg):
        self.image_left = msg
        self.try_combine_images()

    def image_received_right(self, msg):
        self.image_right = msg
        self.try_combine_images()
    
    def image_received_back(self, msg):
        self.image_back = msg
        self.try_combine_images()
    
    def try_combine_images(self):
        if (self.image_front, self.image_right, self.image_left, self.image_back).any() is None:
            return
        
        #TODO

        # compressed_image = bridge.cv2_to_compressed_imgmsg(preview_image)
        # self.combined_image_image_publisher.publish(compressed_image)

def main():
    rclpy.init()
    node = ImageCombiner()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
