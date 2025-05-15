#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from autonav_shared.node import Node
from autonav_shared.types import DeviceState, LogLevel


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

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

        # Perspective transform
        self.src_top_left = (240, 80)
        self.src_top_right = (380, 80)
        self.src_bottom_left = (0, 480)
        self.src_bottom_right = (640, 480)

        self.dest_top_left = (240, 0)
        self.dest_top_right = (380, 0)
        self.dest_bottom_left = (240, 480)
        self.dest_bottom_right = (400, 480)

        # Region of disinterest
        # Order: top-left, top-right, bottom-right, bottom-left
        self.blank_robot = [
            [0, IMAGE_HEIGHT-10],
            [IMAGE_WIDTH, IMAGE_HEIGHT-10],
            [0, IMAGE_HEIGHT],
            [IMAGE_WIDTH, IMAGE_HEIGHT]
        ]

        # Disabling
        self.disable_blur = False
        self.disable_hsv = False
        self.disable_region_of_disinterest = False
        self.disable_perspective_transform = False


class ImageTransformer(Node):
    def __init__(self, dir = "left"):
        super().__init__("autonav_vision_transformer_" + dir) #TODO FIXME does this need to be unique?
        self.config = ImageTransformerConfig()
        self.dir = dir
    
    def apply_config(self, config: dict):
        # HSV
        self.config.lower_hue = config["lower_hue"]
        self.config.lower_sat = config["lower_sat"]
        self.config.lower_val = config["lower_val"]
        self.config.upper_hue = config["upper_hue"]
        self.config.upper_sat = config["upper_sat"]
        self.config.upper_val = config["upper_val"]

        # Blur
        self.config.blur_weight = config["blur_weight"]
        self.config.blur_iterations = config["blur_iterations"]

        # Perspective transform
        self.config.src_top_left = config["src_top_left"]
        self.config.src_top_right = config["src_top_right"]
        self.config.src_bottom_left = config["src_bottom_left"]
        self.config.src_bottom_right = config["src_bottom_right"]

        self.config.dest_top_left = config["dest_top_left"]
        self.config.dest_top_right = config["dest_top_right"]
        self.config.dest_bottom_left = config["dest_bottom_left"]
        self.config.dest_bottom_right = config["dest_bottom_right"]

        # Region of disinterest
        # Order: top-left, top-right, bottom-right, bottom-left
        self.config.blank_robot = config["blank_robot"]

        # Disabling
        self.config.disable_blur = config["disable_blur"]
        self.config.disable_hsv = config["disable_hsv"]
        self.config.disable_region_of_disinterest = config["disable_region_of_disinterest"]
        self.config.disable_perspective_transform = config["disable_perspective_transform"]

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # subscribers
        self.camera_subscriber = self.create_subscription(CompressedImage, f"/autonav/camera/{self.dir}", self.onImageReceived, 1)

        # publishers
        self.camera_filtered_publisher = self.create_publisher(CompressedImage, f"/autonav/vision/filtered/{self.dir}", 1)
        self.camera_debug_publisher = self.create_publisher(CompressedImage, f"/autonav/vision/debug/{self.dir}", 1)

        self.set_device_state(DeviceState.READY)

        self.has_logged = False

        if self.dir == "front":
            self.log("STARTED!", LogLevel.ERROR)


    # Blur
    def apply_blur(self, img):
        if self.config.disable_blur:
            return img
        
        for _ in range(self.config.blur_iterations):
            img = cv2.blur(img, (self.config.blur_weight, self.config.blur_weight))

        return img

    # Threshold
    def apply_hsv(self, img):
        if self.config.disable_hsv:
            return img

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = (self.config.lower_hue, self.config.lower_sat, self.config.lower_val)
        upper = (self.config.upper_hue, self.config.upper_sat, self.config.upper_val)
        mask = cv2.inRange(img, lower, upper)

        return 255 - mask

    # Blank out the robot in the middle of the image
    def apply_region_of_disinterest(self, img):
        if self.config.disable_region_of_disinterest:
            return img

        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, [np.array(self.config.blank_robot)], 0)
        
        return cv2.bitwise_and(img, mask)

    def apply_perspective_transform(self, img, debug=False):
        if self.config.disable_perspective_transform:
            return img
        
        src_pts = np.float32([
            self.config.src_top_left,
            self.config.src_top_right,
            self.config.src_bottom_left,
            self.config.src_bottom_right
        ])

        dest_pts = np.float32([
            self.config.dest_top_left,
            self.config.dest_top_right,
            self.config.dest_bottom_left,
            self.config.dest_bottom_right
        ])

        matrix = cv2.getPerspectiveTransform(src_pts, dest_pts)

        if debug:
            # alpha channel (0, 0, 0, 1) is important for combining images later
            flattened = cv2.warpPerspective(img, matrix, (640, 480), borderValue=(0, 0, 0, 1))
        else:
            flattened = cv2.warpPerspective(img, matrix, (640, 480))

        return flattened

    def onImageReceived(self, msg: CompressedImage):
        # if self.dir == "front":
        #     self.log("RECEIVING!", LogLevel.DEBUG)

        if (self.get_device_state() != DeviceState.OPERATING):
            self.set_device_state(DeviceState.OPERATING)

        # Decompress image
        image = bridge.compressed_imgmsg_to_cv2(msg)

        # separate "filtered_image" variable because we have to do some filtering/drawing on the debug image too
        filtered_image = self.apply_blur(image)
        filtered_image = self.apply_hsv(filtered_image)
        filtered_image = self.apply_region_of_disinterest(filtered_image)
        filtered_image = self.apply_perspective_transform(filtered_image)

        # debug image gets disinterest and warpPerspective
        image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA) # transparency so it combines nicely later on
        image = self.apply_region_of_disinterest(image)
        image = self.apply_perspective_transform(image, True)

        # rotate the image depending on which direction it's facing
        # this step is done last because we don't want to rotate it prior to filtering,
        # so that any side of the robot can be the front.
        # we rotate it now solely for running feelers on the combined image later
        if self.dir == "left":
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        elif self.dir == "right":
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_CLOCKWISE)
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            
        elif self.dir == "back":
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_180)
            image = cv2.rotate(image, cv2.ROTATE_180)

        # publish filtered image
        self.camera_filtered_publisher.publish(bridge.cv2_to_compressed_imgmsg(filtered_image))

        # publish debug image
        self.camera_debug_publisher.publish(bridge.cv2_to_compressed_imgmsg(image))

        # if self.dir == "front":
        #     self.log("transformations publishing!", LogLevel.DEBUG)
    
        if not self.has_logged:
            self.log(f"WE RAN! {self.dir}", LogLevel.ERROR)
            self.has_logged = True

def main():
    rclpy.init()
    
    nodes = []

    for direction in ["left", "right", "front", "back"]:
        nodes.append(ImageTransformer(direction))

    executor = rclpy.executors.MultiThreadedExecutor()
    
    for node in nodes:
        executor.add_node(node)

    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()