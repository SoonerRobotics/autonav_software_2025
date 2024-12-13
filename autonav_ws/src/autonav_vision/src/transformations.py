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
        self.dir = dir

        self.write_config(ImageTransformerConfig())

    def directionify(self, topic):
        return topic + "/" + self.dir

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # subscribers
        self.camera_subscriber = self.create_subscription(CompressedImage, self.directionify("/autonav/camera"), self.onImageReceived, 1)

        # publishers
        self.camera_filtered_publisher = self.create_publisher(CompressedImage, self.directionify("/autonav/vision/filtered"), 1)
        self.camera_debug_publisher = self.create_publisher(CompressedImage, self.directionify("/autonav/vision/debug"), 1)

        self.set_device_state(DeviceState.READY)

    def onImageReceived(self, msg: CompressedImage):
        if (self.get_device_state() != DeviceState.OPERATING):
            self.set_device_state(DeviceState.OPERATING)

        # Decompress image
        image = bridge.compressed_imgmsg_to_cv2(msg)

        filtered_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        filtered_image = cv2.inRange(filtered_image, (self.config["lower_hue"], self.config["lower_sat"], self.config["lower_val"]), (self.config["upper_hue"], self.config["upper_sat"], self.config["upper_val"]))
        filtered_image = 255 - filtered_image

        #TODO

        # rotate the image depending on which direction it's facing
        # this step is done last because we don't want to rotate it prior to filtering,
        # so that any side of the robot can be the front.
        # we rotate it now solely for running feelers on the combined image later
        if self.dir == "left":
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # filtered_image = cv2.resize(filtered_image, (640, 480)) #TODO make it so we don't have to resize
            # image = cv2.resize(image, (640, 480))
        elif self.dir == "right":
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_CLOCKWISE)
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            
            # filtered_image = cv2.resize(filtered_image, (640, 480))
            # image = cv2.resize(image, (640, 480))
        elif self.dir == "back":
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_180)
            image = cv2.rotate(image, cv2.ROTATE_180)

        # publish filtered image
        self.camera_filtered_publisher.publish(bridge.cv2_to_compressed_imgmsg(filtered_image))

        # publish debug image TODO
        self.camera_debug_publisher.publish(bridge.cv2_to_compressed_imgmsg(image))

def main():
    rclpy.init()
    
    nodes = []

    for direction in ["front", "left", "right", "back"]:
        nodes.append(ImageTransformer(direction))

    executor = rclpy.executors.MultiThreadedExecutor()
    
    for node in nodes:
        executor.add_node(node)

    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
