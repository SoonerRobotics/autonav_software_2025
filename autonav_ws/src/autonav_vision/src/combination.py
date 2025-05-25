#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from autonav_shared.node import Node
from autonav_shared.types import DeviceState, LogLevel


bridge = CvBridge()

#TODO all image dimensions should be based off the image received instead of constants in this file, so we only have to change it in like camera.py and then make no changes here

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

#TODO use like max() or min() in case we play with the dimensions
# or like, combine them better so the total combined image is only 640x480 or something
# COMBINED_IMAGE_WIDTH = IMAGE_HEIGHT*2 # left and right cameras are 480 pixels tall but turned on their sides so it's 480 pixels wide and then next to each other, and the top/bottom cameras are only 640 wide so this is fine
COMBINED_IMAGE_WIDTH = IMAGE_HEIGHT*2 + IMAGE_WIDTH # make it a square, follow what the height is
COMBINED_IMAGE_HEIGHT = IMAGE_HEIGHT*2 + IMAGE_WIDTH # top and bottom cameras are 480 pixels tall, plus the left/right cameras turned on their side which adds 640 pixels

class CombinationNodeConfig:
    def __init__(self):
        self.x_offset = (COMBINED_IMAGE_WIDTH//2)-(IMAGE_WIDTH//2)
        self.x_shrink = 240
        self.y_shrink = 240

class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_vision_combiner")
        self.config = CombinationNodeConfig()
    
    def apply_config(self, cfg: dict):
        self.config.x_offset = cfg["x_offset"]
        self.config.x_shrink = cfg["x_shrink"]
        self.config.y_shrink = cfg["y_shrink"]

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        self.oneCamera = True

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


    def combine_debug_images(self):
        # self.log("COMBINING DEBUG...", LogLevel.WARN)

        #TODO FIXME I don't like having a lot of duplicated code here it's really bad form and annoying to update
        # make big combined image to hold them all
        combined_image = np.zeros((COMBINED_IMAGE_HEIGHT, COMBINED_IMAGE_WIDTH, 3), dtype=np.uint8)

        # convert everything to actual images
        #TODO move this out to the actual subscriber callbacks???
        debug_image_front = bridge.compressed_imgmsg_to_cv2(self.debug_image_front)
        debug_image_left = bridge.compressed_imgmsg_to_cv2(self.debug_image_left)
        debug_image_right = bridge.compressed_imgmsg_to_cv2(self.debug_image_right)
        debug_image_back = bridge.compressed_imgmsg_to_cv2(self.debug_image_back)

        # make the bigImages that will hold everything 
        bigImageFront = np.zeros_like(combined_image)
        bigImageLeft = np.zeros_like(combined_image)
        bigImageRight = np.zeros_like(combined_image)
        bigImageBack = np.zeros_like(combined_image)

        # https://stackoverflow.com/questions/40895785/using-opencv-to-overlay-transparent-image-onto-another-image
        # we don't want to clip the images when we move them inwards using x_shrink and y_shrink
        # put each individual transformed camera image (left, right, etc) into its own big combined image with just itself
        bigImageFront[0 + self.config.y_shrink : IMAGE_HEIGHT + self.config.y_shrink, self.config.x_offset : self.config.x_offset+IMAGE_WIDTH] = debug_image_front
        bigImageLeft[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, 0 + self.config.x_shrink : IMAGE_HEIGHT + self.config.x_shrink] = debug_image_left
        bigImageRight[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, -IMAGE_HEIGHT - self.config.x_shrink : -self.config.x_shrink] = debug_image_right
        bigImageBack[COMBINED_IMAGE_HEIGHT-IMAGE_HEIGHT - self.config.y_shrink : -self.config.y_shrink, self.config.x_offset : self.config.x_offset+IMAGE_WIDTH] = debug_image_back

        # get masks for each of those big images
        mask_front = cv2.inRange(bigImageFront, (1,1,1,0), (255, 255, 255, 255))
        mask_left = cv2.inRange(bigImageLeft, (1,1,1,0), (255, 255, 255, 255))
        mask_right = cv2.inRange(bigImageRight, (1,1,1,0), (255, 255, 255, 255))
        mask_back = cv2.inRange(bigImageBack, (1,1,1,0), (255, 255, 255, 255))

        # https://docs.opencv.org/4.x/d0/d86/tutorial_py_image_arithmetics.html
        # https://stackoverflow.com/questions/44333605/what-does-bitwise-and-operator-exactly-do-in-opencv
        # overlay all the individual images (bigImageFront, etc) onto the final image (combined_image) using the masks
        combined_image = cv2.bitwise_or(bigImageFront, combined_image, mask_front)
        combined_image = cv2.bitwise_or(bigImageLeft, combined_image, mask_left)
        combined_image = cv2.bitwise_or(bigImageRight, combined_image, mask_right)
        combined_image = cv2.bitwise_or(bigImageBack, combined_image, mask_back)

        # reset images now that we've combined
        self.debug_image_front = None
        self.debug_image_left = None
        self.debug_image_right = None
        self.debug_image_back = None

        return combined_image

    def try_combine_images(self):
        # if only one camera, only check one camera
        if self.oneCamera and self.image_front is not None and self.debug_image_front is not None:
            # make fakes for all the remaining images
            self.image_right = np.zeros_like(self.image_front)
            self.image_left = np.zeros_like(self.image_front)
            self.image_back = np.zeros_like(self.image_front)
            self.debug_image_right = np.zeros_like(self.debug_image_front)
            self.debug_image_left = np.zeros_like(self.debug_image_front)
            self.debug_image_back = np.zeros_like(self.debug_image_front)

        # this is a horrendous line of code pls don't actually do it this way FIXME
        else:
            if self.image_front is None or self.image_right is None or self.image_left is None or self.image_back is None or self.debug_image_front is None or self.debug_image_right is None or self.debug_image_left is None or self.debug_image_back is None:
                return
        
        # self.log("COMBINING ALL...", LogLevel.ERROR)

        # essentially what we need to do is:
        #  - make a big blank combined_image
        #  - blit the warpped image into its position on the big image
        #  - make masks for all the big images using alpha transparency or whatever (for the debug images)
        #  - binary_or all of the images/masks together
        # that is gonna require 4 separate like, big blank images and stuff
        # for reference view autonav_feelers/src/vision_test.py
        # https://stackoverflow.com/questions/14063070/overlay-a-smaller-image-on-a-larger-image-python-opencv
        # the basic idea is to have the front camera on top, then the left and right cameras on their sides beneath that, and the back camera underneath that,
        # so that it kind of looks top-down or like some weird 360 degree camera.
        combined_image = np.zeros((COMBINED_IMAGE_HEIGHT, COMBINED_IMAGE_WIDTH))
        
        # convert all the images 
        # TODO move this to individual callbacks?
        mask_front = bridge.compressed_imgmsg_to_cv2(self.image_front)
        mask_left = bridge.compressed_imgmsg_to_cv2(self.image_left)
        mask_right = bridge.compressed_imgmsg_to_cv2(self.image_right)
        mask_back = bridge.compressed_imgmsg_to_cv2(self.image_back)

        # make the 4 big images
        bigImageFront = np.zeros_like(combined_image)
        bigImageLeft = np.zeros_like(combined_image)
        bigImageRight = np.zeros_like(combined_image)
        bigImageBack = np.zeros_like(combined_image)

        # https://stackoverflow.com/questions/40895785/using-opencv-to-overlay-transparent-image-onto-another-image
        # we don't want to clip the images when we move them inwards using x_shrink and y_shrink
        # put each individual transformed camera image (left, right, etc) into its own big combined image with just itself
        bigImageFront[0 + self.config.y_shrink : IMAGE_HEIGHT + self.config.y_shrink, self.config.x_offset : self.config.x_offset+IMAGE_WIDTH] = mask_front
        bigImageLeft[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, 0 + self.config.x_shrink : IMAGE_HEIGHT + self.config.x_shrink] = mask_left
        bigImageRight[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, -IMAGE_HEIGHT - self.config.x_shrink : -self.config.x_shrink] = mask_right
        bigImageBack[COMBINED_IMAGE_HEIGHT-IMAGE_HEIGHT - self.config.y_shrink : -self.config.y_shrink, self.config.x_offset : self.config.x_offset+IMAGE_WIDTH] = mask_back

        # https://docs.opencv.org/4.x/d0/d86/tutorial_py_image_arithmetics.html
        # https://stackoverflow.com/questions/44333605/what-does-bitwise-and-operator-exactly-do-in-opencv
        # overlay all the individual images (bigImageFront, etc) onto the final image (combined_image) using the masks
        combined_image = cv2.bitwise_or(bigImageFront, combined_image, mask_front)
        combined_image = cv2.bitwise_or(bigImageLeft, combined_image, mask_left)
        combined_image = cv2.bitwise_or(bigImageRight, combined_image, mask_right)
        combined_image = cv2.bitwise_or(bigImageBack, combined_image, mask_back)

        # reset the images now that we've combined a frame
        self.image_front = None
        self.image_left = None
        self.image_right = None
        self.image_back = None

        # run the debug combiner
        debug_combined = self.combine_debug_images()

        # publish both images
        self.combined_image_publisher.publish(bridge.cv2_to_compressed_imgmsg(combined_image))
        self.combined_debug_image_publisher.publish(bridge.cv2_to_compressed_imgmsg(debug_combined))

def main():
    rclpy.init()

    node = ImageCombiner()
    rclpy.spin(node)

    cv2.destroyAllWindows() # just in case

    rclpy.shutdown()


if __name__ == "__main__":
    main()
