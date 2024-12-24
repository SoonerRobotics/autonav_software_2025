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

#TODO all image dimensions should be based off the image received instead of constants in this file, so we only have to change it in like camera.py and then make no changes here

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

#TODO use like max() or min() in case we play with the dimensions
# or like, combine them better so the total combined image is only 640x480 or something
# COMBINED_IMAGE_WIDTH = IMAGE_HEIGHT*2 # left and right cameras are 480 pixels tall but turned on their sides so it's 480 pixels wide and then next to each other, and the top/bottom cameras are only 640 wide so this is fine
COMBINED_IMAGE_WIDTH = IMAGE_HEIGHT*2 + IMAGE_WIDTH # make it a square, follow what the height is
COMBINED_IMAGE_HEIGHT = IMAGE_HEIGHT*2 + IMAGE_WIDTH # top and bottom cameras are 480 pixels tall, plus the left/right cameras turned on their side which adds 640 pixels


class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_vision_combiner")

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        self.image_front = None
        self.image_left = None
        self.image_right = None
        self.image_back = None

        self.debug_image_front = None
        self.debug_image_left = None
        self.debug_image_right = None
        self.debug_image_back = None

        self.image_front_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/front", self.image_received_front, 1)
        self.image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/left", self.image_received_left, 1)
        self.image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/right", self.image_received_right, 1)
        self.image_back_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/filtered/back", self.image_received_back, 1)

        self.debug_image_front_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/front", self.debug_image_received_front, 1)
        self.debug_image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/left", self.debug_image_received_left, 1)
        self.debug_image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/right", self.debug_image_received_right, 1)
        self.debug_image_back_subscriber = self.create_subscription(CompressedImage, "/autonav/vision/debug/back", self.debug_image_received_back, 1)

        self.combined_image_publisher = self.create_publisher(CompressedImage, "/autonav/vision/combined/filtered", 1)
        self.combined_debug_image_publisher = self.create_publisher(CompressedImage, "/autonav/vision/combined/debug", 1)

        self.feeler_debug_image_subscriber = self.create_subscription(CompressedImage, "/autonav/feelers/debug", self.on_feelers_received, 1) # TEMP TODO FIXME
        self.feeler_debug_image = None

        self.set_device_state(DeviceState.READY)

        #FIXME TEMP DEBUG HACK
        self.log("starting image combiner...", LogLevel.WARN)
        self.video_writer = cv2.VideoWriter("./data/combined.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (COMBINED_IMAGE_WIDTH, COMBINED_IMAGE_HEIGHT)) #TODO
        self.debug_video_writer = cv2.VideoWriter("./data/debug_combined.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (COMBINED_IMAGE_WIDTH, COMBINED_IMAGE_HEIGHT)) #TODO
        self.feeler_video_writer = cv2.VideoWriter("./data/debug_feeler.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (COMBINED_IMAGE_WIDTH, COMBINED_IMAGE_HEIGHT)) #TODO
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
    
    # ===
    def debug_image_received_front(self, msg):
        self.debug_image_front = msg
        self.try_combine_images()

    def debug_image_received_left(self, msg):
        self.debug_image_left = msg
        self.try_combine_images()

    def debug_image_received_right(self, msg):
        self.debug_image_right = msg
        self.try_combine_images()
    
    def debug_image_received_back(self, msg):
        self.debug_image_back = msg
        self.try_combine_images()
    # ===

    def try_combine_images(self):
        # this is a horrendous line of code pls don't actually do it this way FIXME
        if self.image_front is None or self.image_right is None or self.image_left is None or self.image_back is None or self.debug_image_front is None or self.debug_image_right is None or self.debug_image_left is None or self.debug_image_back is None:
            return
        
        combined = np.zeros((COMBINED_IMAGE_HEIGHT, COMBINED_IMAGE_WIDTH))
        
        image_front = bridge.compressed_imgmsg_to_cv2(self.image_front)
        image_left = bridge.compressed_imgmsg_to_cv2(self.image_left)
        image_right = bridge.compressed_imgmsg_to_cv2(self.image_right)
        image_back = bridge.compressed_imgmsg_to_cv2(self.image_back)

        # we have a copy of every image now, so combine them
        # https://stackoverflow.com/questions/14063070/overlay-a-smaller-image-on-a-larger-image-python-opencv
        # the basic idea is to have the front camera on top, then the left and right cameras on their sides beneath that, and the back camera underneath that,
        # so that it kind of looks top-down or like some weird 360 degree camera.
        x_offset = (COMBINED_IMAGE_WIDTH//2)-(IMAGE_WIDTH//2)
        combined[0 : IMAGE_HEIGHT, x_offset : x_offset+IMAGE_WIDTH] = image_front
        combined[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, 0 : IMAGE_HEIGHT] = image_left # the left and right cameras are rotated 90 degrees so coordinates are backwards, it is not [IMAGE_HEIGHT : IMAGE_HEIGHT*2, 0 : IMAGE_WIDTH]
        combined[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, -IMAGE_HEIGHT : ] = image_right # same here, if it wasn't rotated it would be [IMAGE_HEIGHT : IMAGE_HEIGHT*2, IMAGE_WIDTH : ]
        combined[COMBINED_IMAGE_HEIGHT-IMAGE_HEIGHT : , x_offset : x_offset+IMAGE_WIDTH] = image_back

        # and publish the image
        self.combined_image_publisher.publish(bridge.cv2_to_compressed_imgmsg(combined))

        # reset the images now that we've combined a frame
        self.image_front = None
        self.image_left = None
        self.image_right = None
        self.image_back = None

        #TODO FIXME I don't like having a lot of duplicated code here it's really bad form and annoying to update
        debug_image_front = bridge.compressed_imgmsg_to_cv2(self.debug_image_front)
        debug_image_left = bridge.compressed_imgmsg_to_cv2(self.debug_image_left)
        debug_image_right = bridge.compressed_imgmsg_to_cv2(self.debug_image_right)
        debug_image_back = bridge.compressed_imgmsg_to_cv2(self.debug_image_back)

        debug_combined = np.zeros((COMBINED_IMAGE_HEIGHT, COMBINED_IMAGE_WIDTH, 3), dtype=np.uint8)
        # debug_combined = cv2

        # we have a copy of every image now, so combine them
        #FIXME *really* don't like duplicated code here
        x_offset = (COMBINED_IMAGE_WIDTH//2)-(IMAGE_WIDTH//2)
        debug_combined[0 : IMAGE_HEIGHT, x_offset : x_offset+IMAGE_WIDTH] = debug_image_front
        debug_combined[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, 0 : IMAGE_HEIGHT] = debug_image_left # the left and right cameras are rotated 90 degrees so coordinates are backwards, it is not [IMAGE_HEIGHT : IMAGE_HEIGHT*2, 0 : IMAGE_WIDTH]
        debug_combined[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, IMAGE_HEIGHT+IMAGE_WIDTH : ] = debug_image_right # same here, if it wasn't rotated it would be [IMAGE_HEIGHT : IMAGE_HEIGHT*2, IMAGE_WIDTH : ]
        debug_combined[COMBINED_IMAGE_HEIGHT-IMAGE_HEIGHT : , x_offset : x_offset+IMAGE_WIDTH] = debug_image_back

        # and publish the image
        self.combined_debug_image_publisher.publish(bridge.cv2_to_compressed_imgmsg(debug_combined))

        # reset the images now that we've combined a frame
        self.debug_image_front = None
        self.debug_image_left = None
        self.debug_image_right = None
        self.debug_image_back = None

        #TEMP TODO FIXME
        feeler_image = None
        if (self.feeler_debug_image != None):
            feeler_image = bridge.compressed_imgmsg_to_cv2(self.feeler_debug_image)
            self.feeler_debug_image = None
            if not self.has_logged:
                self.log(f"Shape of feeler_image: {feeler_image.shape}")
                self.has_logged = True

        # FIXME DEBUG HACK
        # while the UI still in development, log images to a video for debugging
        if self.frame < 500:
            combined = cv2.cvtColor(np.uint8(combined), cv2.COLOR_GRAY2BGR)
            self.video_writer.write(combined)
            self.debug_video_writer.write(debug_combined)
            self.feeler_video_writer.write(feeler_image)
        elif self.video_writer.isOpened():
            self.video_writer.release()
            self.debug_video_writer.release()
            self.feeler_video_writer.release()
            self.log("combined image logger is done!", LogLevel.ERROR)
        
        self.frame += 1
        # self.log(f"combining frame {self.frame}. . .", LogLevel.WARN)
    
    def on_feelers_received(self, msg):
        self.feeler_debug_image = msg #TEMP TODO FIXME

def main():
    rclpy.init()

    node = ImageCombiner()
    rclpy.spin(node)

    cv2.destroyAllWindows() # just in case

    rclpy.shutdown()


if __name__ == "__main__":
    main()
