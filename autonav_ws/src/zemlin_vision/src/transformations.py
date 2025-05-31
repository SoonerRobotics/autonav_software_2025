#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState

g_bridge = CvBridge()
g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0

MAP_RES = 80


class ImageTransformerConfig:
    def __init__(self):
        self.lower_hue = 0
        self.lower_saturation = 0
        self.lower_value = 0
        self.upper_hue = 255
        self.upper_saturation = 180
        self.upper_value = 210
        self.blur = 5
        self.blur_iterations = 3
        self.region_of_disinterest_offset = 30


class ImageTransformer(Node):
    def __init__(self):
        super().__init__("autonav_vision_transformer")
        self.config = ImageTransformerConfig()

    def init(self):
        self.cameraSubscriber = self.create_subscription(CompressedImage, "/autonav/camera/front", self.onImageReceived, 1)
        self.rawMapPublisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", 1)
        self.filteredImagePublisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/image", 1)
        self.set_device_state(DeviceState.OPERATING)

    def apply_config(self, config):
        self.log("Applying new configuration to ImageTransformer")
        self.config.lower_hue = config["lower_hue"]
        self.config.lower_saturation = config["lower_saturation"]
        self.config.lower_value = config["lower_value"]
        self.config.upper_hue = config["upper_hue"]
        self.config.upper_saturation = config["upper_saturation"]
        self.config.upper_value = config["upper_value"]
        self.config.blur = config["blur"]
        self.config.blur_iterations = config["blur_iterations"]
        self.config.region_of_disinterest_offset = config["region_of_disinterest_offset"]

    def getBlur(self):
        blur = self.config.blur
        blur = max(1, blur)
        return (blur, blur)

    def regionOfDisinterest(self, img, vertices):
        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, vertices, 0)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def flattenImage(self, img):
        top_left = (int)(img.shape[1] * 0.26), (int)(img.shape[0])
        top_right = (int)(img.shape[1] - img.shape[1] * 0.26), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
        dest_pts = np.float32([ [0, 480], [640, 480] ,[0, 0], [640, 0]])

        matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)
        output = cv2.warpPerspective(img, matrix, (640, 480))
        return output

    def publishOccupancyMap(self, img):
        datamap = cv2.resize(img, dsize=(MAP_RES, MAP_RES), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.rawMapPublisher.publish(msg)

    def onImageReceived(self, image: CompressedImage):
        self.perf_start("Image Transformation")

        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image)

        # Blur it up
        for _ in range(self.config.blur_iterations):
            cv_image = cv2.blur(cv_image, self.getBlur())

        # Apply filter and return a mask
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = (
            self.config.lower_hue,
            self.config.lower_saturation,
            self.config.lower_value
        )
        upper = (
            self.config.upper_hue,
            self.config.upper_saturation,
            self.config.upper_value
        )
        mask1 = cv2.inRange(img, lower, upper)

        lower = (150, 45, 40)
        upper = (220, 255, 255)
        mask2 = cv2.inRange(img, lower, upper)
        mask = cv2.add(mask1, mask2)
        mask = 255 - mask

        # Apply region of disinterest and flattening
        height = img.shape[0]
        width = img.shape[1]
        region_of_disinterest_vertices=[
            (0, height),
            ((width / 2) - 200, height / 2 + self.config.region_of_disinterest_offset),
            ((width / 2) + 200, height / 2 + self.config.region_of_disinterest_offset),
            (width, height)
        ]
        mask = self.regionOfDisinterest(mask, np.array([region_of_disinterest_vertices], np.int32))
        mask[mask < 250] = 0

        mask = self.flattenImage(mask)

        preview_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cv2.polylines(preview_image, np.array([region_of_disinterest_vertices], np.int32), True, (0, 255, 0), 2)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        preview_msg.header = image.header
        preview_msg.format = "jpeg"
        self.filteredImagePublisher.publish(preview_msg)

        # Actually generate the map
        self.publishOccupancyMap(mask)

        self.perf_stop("Image Transformation")


def main():
    rclpy.init()
    rclpy.spin(ImageTransformer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
