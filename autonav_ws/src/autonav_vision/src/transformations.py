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

class ImageTransformerConfig:
    def __init__(self):
        # HSV
        self.lower_hue = 0
        self.lower_sat = 0
        self.lower_val = 0
        self.upper_hue = 255
        self.upper_sat = 95
        self.upper_val = 210

        # Blur
        self.blur_weight = 5
        self.blur_iterations = 3
        self.map_res = 80

        # Perspective transform
        self.left_topleft = [80, 220]
        self.left_topright = [400, 220]
        self.left_bottomright = [480, 640]
        self.left_bottomleft = [0, 640]
        self.right_topleft = [80, 220]
        self.right_topright = [400, 220]
        self.right_bottomright = [480, 640]
        self.right_bottomleft = [0, 640]

        # Region of disinterest
        # Order: top-left, top-right, bottom-right, bottom-left
        self.parallelogram_left = [[500, 405], [260, 400], [210, 640], [640, 640]]
        self.parallelogram_right = [[0, 415], [195, 390], [260, 640], [0, 640]]

        self.top_width = 320
        self.bottom_width = 240
        self.offset = 20

        # Disabling
        self.disable_blur = False
        self.disable_hsv = False
        self.disable_region_of_disinterest = False
        self.disable_perspective_transform = False


class ImageTransformer(Node):
    def __init__(self, dir = "left"):
        super().__init__("autonav_vision_transformer")
        self.config = self.get_default_config()
        self.dir = dir

    def directionify(self, topic):
        return topic + "/" + self.dir

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # subscribers
        self.camera_subscriber = self.create_subscription(CompressedImage, self.directionify("/autonav/camera/"), self.onImageReceived)

        # publishers (filtered view)
        self.camera_debug_publisher = self.create_publisher(CompressedImage, self.directionify("/autonav/vision/filtered") + "/cutout")

        self.write_config(ImageTransformerConfig())

        self.set_device_state(DeviceState.READY)

    def onImageReceived(self, image: CompressedImage):
        if (self.device_state != DeviceState.OPERATING):
            self.set_device_state(DeviceState.OPERATING)

        # Decompress image
        img = bridge.compressed_imgmsg_to_cv2(image)

        #TODO

        # publish filtered image
        self.camera_debug_publisher.publish(bridge.cv2_to_compressed_imgmsg(img))

def main():
    rclpy.init()
    node_front = ImageTransformer(dir = "front")
    node_left = ImageTransformer(dir = "left")
    node_right = ImageTransformer(dir = "right")
    node_back = ImageTransformer(dir = "back")
    Node.run_nodes([node_front, node_left, node_right, node_back])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
