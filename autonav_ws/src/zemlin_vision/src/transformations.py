#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

from autonav_shared.node import Node
from autonav_shared.types import DeviceState


g_bridge = CvBridge()
g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0

MAP_RES = 80


class TransformationsConfig:
    def __init__(self):
        self.lower_hue = 0
        self.lower_saturation = 0
        self.lower_value = 0
        self.upper_hue = 255
        self.upper_saturation = 95
        self.upper_value = 230
        self.flatten_param = 0.3
        self.blur = 5
        self.blur_iterations = 3
        self.region_of_disinterest_offset = 45


class ImageTransformer(Node):
    def __init__(self):
        super().__init__("zemlin_vision_transformations")

        self.config = TransformationsConfig()

    def apply_config(self, config):
        self.config.lower_hue = config["lower_hue"]
        self.config.lower_saturation = config["lower_saturation"]
        self.config.lower_value = config["lower_value"]
        self.config.upper_hue = config["upper_hue"]
        self.config.upper_saturation = config["upper_saturation"]
        self.config.upper_value = config["upper_value"]
        self.config.blur = config["blur"]
        self.config.blur_iterations = config["blur_iterations"]
        self.config.region_of_disinterest_offset = config["region_of_disinterest_offset"]
        self.config.flatten_param = config["flatten_param"]

    def init(self):
        self.camera_sub = self.create_subscription(CompressedImage, "/autonav/camera/front", self.on_image_received, 1)
        self.raw_map_pub = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", 1)
        self.filtered_pub = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/image", 1)

        self.set_device_state(DeviceState.OPERATING)

    def get_blur(self):
        blur = self.config.blur
        blur = max(1, blur)
        return (blur, blur)

    def region_of_interest(self, img, vertices):
        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, vertices, 0)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def flatten(self, img):
        top_left = (int)(img.shape[1] * self.config.flatten_param), (int)(img.shape[0])
        top_right = (int)(img.shape[1] - img.shape[1] * self.config.flatten_param), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
        dest_pts = np.float32([[0, 400], [640, 400], [200, 0], [440, 0]])

        matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)
        output = cv2.warpPerspective(img, matrix, (640, 480))
        return output

    def publish_map(self, img):
        datamap = cv2.resize(img, dsize=(MAP_RES, MAP_RES), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.raw_map_pub.publish(msg)

    def on_image_received(self, image: CompressedImage):
        self.perf_start("transformations")
        
        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image)

        # Blur it up
        for _ in range(self.config.blur_iterations):
            cv_image = cv2.blur(cv_image, self.get_blur())

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
        mask = cv2.inRange(img, lower, upper)
        mask = 255 - mask

        # Apply region of disinterest and flattening
        height = img.shape[0]
        width = img.shape[1]
        # region_of_disinterest_vertices=[
        #     (0, height),
        #     (width / 2, height / 2 + self.config.region_of_disinterest_offset),
        #     (width, height)
        # ]
        # mask = self.region_of_interest(mask, np.array([region_of_disinterest_vertices], np.int32))
        # mask[mask < 250] = 0
        
        # Cut off a 50 by 50 px square from the bottom center
        vertices = [
            (0, height),
            (width / 2 - 50, height / 2 + self.config.region_of_disinterest_offset),
            (width / 2 + 50, height / 2 + self.config.region_of_disinterest_offset),
            (width, height)
        ]
        mask = self.region_of_interest(mask, np.array([vertices], np.int32))

        mask = self.flatten(mask)

        # Actually generate the map
        self.publish_map(mask)
        
        preview_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        preview_msg.header = image.header
        preview_msg.format = "jpeg"
        self.filtered_pub.publish(preview_msg)

        self.perf_stop("transformations")

def main():
    rclpy.init()
    Node.run_node(ImageTransformer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()