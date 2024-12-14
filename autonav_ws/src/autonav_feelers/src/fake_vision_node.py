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

#TODO use like max() or min() in case we play with the dimensions
# or like, combine them better so the total combined image is only 640x480 or something
COMBINED_IMAGE_WIDTH = IMAGE_HEIGHT*2 # left and right cameras are 480 pixels tall but turned on their sides so it's 480 pixels wide and then next to each other, and the top/bottom cameras are only 640 wide so this is fine
COMBINED_IMAGE_HEIGHT = IMAGE_HEIGHT*2 + IMAGE_WIDTH # top and bottom cameras are 480 pixels tall, plus the left/right cameras turned on their side which adds 640 pixels


class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_vision_combiner")

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # subscribers
        self.feeler_debug_image_subscriber = self.create_subscription(CompressedImage, "/autonav/feelers/debug", self.on_feelers_received, 1) # TEMP TODO FIXME
        self.feeler_debug_image = None

        self.feeler_video_writer = cv2.VideoWriter("./data/debug_feeler_test.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (COMBINED_IMAGE_WIDTH, COMBINED_IMAGE_HEIGHT)) #TODO

        # publishers
        self.combined_image_publisher = self.create_publisher(CompressedImage, "/autonav/vision/combined/filtered", 1)
        self.video_reader = cv2.VideoCapture("./data/combined.mp4")

        self.combined_debug_image_publisher = self.create_publisher(CompressedImage, "/autonav/vision/combined/debug", 1)
        self.debug_video_reader = cv2.VideoCapture("./data/debug_combined.mp4")

        #TODO make timers
        self.publish_timer = self.create_timer(5, self.publish_combined_images)

        self.frame = 0
        
        self.log("starting image combiner...", LogLevel.WARN)
        self.set_device_state(DeviceState.READY)


    # just write that straight to output
    def on_feelers_received(self, msg):
        self.feeler_video_writer.write(bridge.compressed_imgmsg_to_cv2(msg))

    def publish_combined_images(self):
        print(f"Publishing frame: {self.frame}")

        ret1, combined_image = self.video_reader.read()
        ret2, debug_image = self.debug_video_reader.read()

        if not ret1 or not ret2 or self.frame > 3:
            self.log("OUT OF IMAGES", LogLevel.FATAL)

            self.video_reader.release()
            self.debug_video_reader.release()
            self.feeler_video_writer.release()

            raise SystemExit

        self.frame += 1

        self.combined_image_publisher.publish(bridge.cv2_to_compressed_imgmsg(combined_image))
        self.combined_debug_image_publisher.publish(bridge.cv2_to_compressed_imgmsg(debug_image))
    

def main():
    rclpy.init()

    node = ImageCombiner()
    rclpy.spin(node)

    node.feeler_video_writer.release()
    cv2.destroyAllWindows() # just in case

    rclpy.shutdown()


if __name__ == "__main__":
    main()
