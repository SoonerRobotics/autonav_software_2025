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

#FIXME TODO
IMAGE_WIDTH = 640*4 # four cameras, so for now just line them up side by side all in a row
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

        #FIXME TEMP DEBUG HACK
        self.log("starting image combiner...", LogLevel.WARN)
        self.video_writer = cv2.VideoWriter("./data/combined.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (IMAGE_WIDTH, IMAGE_HEIGHT)) #TODO
        self.frame = 0
        self.has_logged = False

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
        # if (self.image_front, self.image_right, self.image_left, self.image_back).any() is None:
        #     return

        if self.image_front is None or self.image_right is None or self.image_left is None or self.image_back is None:
            return
        
        image_front = bridge.compressed_imgmsg_to_cv2(self.image_front)
        image_left = bridge.compressed_imgmsg_to_cv2(self.image_left)
        image_right = bridge.compressed_imgmsg_to_cv2(self.image_right)
        image_back = bridge.compressed_imgmsg_to_cv2(self.image_back)

        # we have a copy of every image now, so combine them
        combined = np.concatenate((image_front, image_left, image_right, image_back), axis=1)

        if not self.has_logged:
            self.log(f"Combined image size is {combined.shape}!")
            self.has_logged = True

        # and publish the image
        self.combined_image_publisher.publish(bridge.cv2_to_compressed_imgmsg(combined))

        # reset the images now that we've combined a frame
        self.image_front = None
        self.image_left = None
        self.image_right = None
        self.image_back = None

        # FIXME DEBUG HACK
        # while the UI still in development, log images to a video for debugging
        if self.frame < 200:
            combined = cv2.cvtColor(np.uint8(combined), cv2.COLOR_GRAY2BGR)
            self.video_writer.write(combined)
        elif self.video_writer.isOpened():
            self.video_writer.release()
            self.log("combined image logger is done!", LogLevel.ERROR)
        
        self.frame += 1
        # self.log(f"combining frame {self.frame}. . .", LogLevel.WARN)

def main():
    rclpy.init()

    node = ImageCombiner()
    rclpy.spin(node)

    cv2.destroyAllWindows() # just in case

    rclpy.shutdown()


if __name__ == "__main__":
    main()
