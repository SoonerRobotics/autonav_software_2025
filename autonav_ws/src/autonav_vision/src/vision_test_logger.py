#!/usr/bin/env python3

import rclpy
import cv2, time
import numpy as np

import rclpy.qos
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState


bridge = CvBridge()

#TODO use like, the actual image dimensions instead of this or something?
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

COMBINED_IMAGE_WIDTH = 1600
COMBINED_IMAGE_HEIGHT = 1600


class VideoLogger(Node):
    def __init__(self):
        super().__init__("autonav_vision_logger")

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # == subscribers ==
        # raw
        self.front_video_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/front", self.on_front_received, 1)
        self.left_video_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/left", self.on_left_received, 1)
        self.right_video_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/right", self.on_right_received, 1)
        self.back_video_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/back", self.on_back_received, 1)

        self.debug_image_front_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/front", self.on_front_debug_received, 1)
        self.debug_image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/left", self.on_left_debug_received, 1)
        self.debug_image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/right", self.on_right_debug_received, 1)
        self.debug_image_back_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/back", self.on_back_debug_received, 1)

        # combined
        self.combined_image_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/combined/filtered", self.on_combined_received, 1)
        self.combined_debug_image_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/combined/debug", self.on_combined_debug_received, 1)

        # feelers
        self.feeler_debug_image_subscriber = self.create_subscription(CompressedImage, "/autonav/feelers/debug", self.on_feelers_received, 1)


        # == writers ==
        # raw
        self.front_video_writer = cv2.VideoWriter("./data/front.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.left_video_writer = cv2.VideoWriter("./data/left.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.right_video_writer = cv2.VideoWriter("./data/right.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.back_video_writer = cv2.VideoWriter("./data/back.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))

        self.front_debug_video_writer = cv2.VideoWriter("./data/front_debug.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.left_debug_video_writer = cv2.VideoWriter("./data/left_debug.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.right_debug_video_writer = cv2.VideoWriter("./data/right_debug.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.back_debug_video_writer = cv2.VideoWriter("./data/back_debug.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT))

        # combined
        self.combined_video_writer = cv2.VideoWriter("./data/combined.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (COMBINED_IMAGE_WIDTH, COMBINED_IMAGE_HEIGHT))
        self.combined_debug_video_writer = cv2.VideoWriter("./data/debug_combined.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (COMBINED_IMAGE_WIDTH, COMBINED_IMAGE_HEIGHT))

        # feeler
        self.feeler_video_writer = cv2.VideoWriter("./data/debug_feeler.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (COMBINED_IMAGE_WIDTH, COMBINED_IMAGE_HEIGHT))

        self.log("starting video logger...", LogLevel.DEBUG)
        self.set_device_state(DeviceState.READY)

    # raw
    def on_front_received(self, msg):
        self.front_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))
    
    def on_left_received(self, msg):
        self.left_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    def on_right_received(self, msg):
        self.right_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    def on_back_received(self, msg):
        self.back_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    # raw debug
    def on_front_debug_received(self, msg):
        self.front_debug_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))
    
    def on_left_debug_received(self, msg):
        self.left_debug_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    def on_right_debug_received(self, msg):
        self.right_debug_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    def on_back_debug_received(self, msg):
        self.back_debug_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    # combined
    def on_combined_received(self, msg):
        self.combined_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    def on_combined_debug_received(self, msg):
        self.combined_debug_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    # feelers
    def on_feelers_received(self, msg):
        self.feeler_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

        self.frame += 1 #TODO FIXME find a way to not have this

    

def main():
    rclpy.init()

    node = VideoLogger()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.front_video_writer.release()
        node.left_video_writer.release()

        node.right_video_writer.release()
        node.back_video_writer.release()

        node.front_debug_video_writer.release()
        node.left_debug_video_writer.release()

        node.right_debug_video_writer.release()
        node.back_debug_video_writer.release()

        node.combined_video_writer.release()

        node.combined_debug_video_writer.release()
        node.feeler_video_writer.release()
        
    # cleanup
    node.front_video_writer.release()
    node.left_video_writer.release()
    node.right_video_writer.release()
    node.back_video_writer.release()
    
    node.front_debug_video_writer.release()
    node.left_debug_video_writer.release()
    node.right_debug_video_writer.release()
    node.back_debug_video_writer.release()

    node.combined_video_writer.release()
    node.combined_debug_video_writer.release()
    node.feeler_video_writer.release()

    cv2.destroyAllWindows() # just in case

    rclpy.shutdown()


if __name__ == "__main__":
    main()
