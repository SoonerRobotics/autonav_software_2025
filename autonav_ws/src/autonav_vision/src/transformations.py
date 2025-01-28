#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import cv2
import numpy as np

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from scr.node import Node
from scr.states import DeviceStateEnum
import json

g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 200  #width for 4 cameras
g_mapData.height = 200  #height for 4 cameras 
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0

class ImageTransformerConfig:
    def __init__(self):
        self.lower_hue = 0
        self.lower_sat = 0
        self.lower_val = 0
        self.upper_hue = 255
        self.upper_sat = 95
        self.upper_val = 210
        self.blur_weight = 5
        self.blur_iterations = 3
        self.map_res = 80

        self.offset = 20
        self.top_width = 320
        self.bottom_width = 240

        self.disable_blur = False
        self.disable_hsv = False
        self.disable_region_of_disinterest = False
        self.disable_perspective_transform = False

        self.parallelogram_left = [[500, 405], [260, 400], [210, 640], [640, 640]]
        self.parallelogram_right = [[0, 415], [195, 390], [260, 640], [0, 640]]
        
        # Camera positions for four cameras (left-top, right-top, left-bottom, right-bottom)
        self.camera_positions = {
            "left_top": {"topleft": [80, 220], "topright": [400, 220], "bottomright": [480, 640], "bottomleft": [0, 640]},
            "right_top": {"topleft": [80, 220], "topright": [400, 220], "bottomright": [480, 640], "bottomleft": [0, 640]},
            "left_bottom": {"topleft": [80, 220], "topright": [400, 220], "bottomright": [480, 640], "bottomleft": [0, 640]},
            "right_bottom": {"topleft": [80, 220], "topright": [400, 220], "bottomright": [480, 640], "bottomleft": [0, 640]},
        }

class ImageTransformer(Node):
    def __init__(self, camera_position):
        super().__init__("autonav_vision_transformer")
        self.config = self.get_default_config()
        self.camera_position = camera_position

    def directionify(self, topic):
        return topic + "/" + self.camera_position

    def init(self):
        self.camera_subscriber = self.create_subscription(
            CompressedImage, self.directionify("/autonav/camera/compressed"),
            self.onImageReceived, self.qos_profile)
        self.camera_debug_publisher = self.create_publisher(
            CompressedImage, self.directionify("/autonav/camera/compressed") + "/cutout", self.qos_profile)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, self.directionify("/autonav/cfg_space/raw"), 1)
        self.grid_image_publisher = self.create_publisher(
            CompressedImage, self.directionify("/autonav/cfg_space/raw/image") + "_small", self.qos_profile)

        self.set_device_state(DeviceStateEnum.OPERATING)

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return ImageTransformerConfig()

    def apply_blur(self, img):
        if self.config.disable_blur:
            return img
        for _ in range(self.config.blur_iterations):
            img = cv2.blur(img, (self.config.blur_weight, self.config.blur_weight))
        return img

    def apply_hsv(self, img):
        if self.config.disable_hsv:
            return img
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = (self.config.lower_hue, self.config.lower_sat, self.config.lower_val)
        upper = (self.config.upper_hue, self.config.upper_sat, self.config.upper_val)
        mask = cv2.inRange(img, lower, upper)
        return 255 - mask

    def apply_region_of_disinterest(self, img):
        if self.config.disable_region_of_disinterest:
            return img
        mask = np.ones_like(img) * 255
        parallelogram = self.config.parallelogram_left if "left" in self.camera_position else self.config.parallelogram_right
        cv2.fillPoly(mask, [np.array(parallelogram)], 0)
        return cv2.bitwise_and(img, mask)

    def apply_perspective_transform(self, img):
        if self.config.disable_perspective_transform:
            return img
        camera_pos = self.config.camera_positions[self.camera_position]
        pts = np.array([
            camera_pos["topleft"],
            camera_pos["topright"],
            camera_pos["bottomright"],
            camera_pos["bottomleft"]
        ], dtype="float32")
        return self.epic_noah_transform(img, pts, self.config.top_width, self.config.bottom_width, 640, self.config.offset)

    def epic_noah_transform(self, image, pts, top_width, bottom_width, height, offset):
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        
        if "left" in self.camera_position:
            dst = np.array([
                [240 - bottom_width - offset, 0],
                [240 - offset, 0],
                [240 - offset, height - 1],
                [240 - top_width - offset, height - 1]
            ], dtype="float32")
        else:
            dst = np.array([
                [240 + offset, 0],
                [240 + bottom_width + offset, 0],
                [240 + top_width + offset, height - 1],
                [240 + offset, height - 1]
            ], dtype="float32")
        M = cv2.getPerspectiveTransform(rect, dst)
        return cv2.warpPerspective(image, M, (maxWidth, maxHeight))

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    def publish_occupancy_grid(self, img):
        datamap = cv2.resize(img, dsize=(self.config.map_res, self.config.map_res), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.grid_publisher.publish(msg)

        preview_image = cv2.resize(img, dsize=(80, 80), interpolation=cv2.INTER_LINEAR)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        self.grid_image_publisher.publish(preview_msg)

    def onImageReceived(self, image: CompressedImage):
        img = g_bridge.compressed_imgmsg_to_cv2(image)

        img = self.apply_blur(img)
        img = self.apply_hsv(img)
        img = self.apply_region_of_disinterest(img)
        img = self.apply_perspective_transform(img)

        self.publish_occupancy_grid(img)

def main():
    rclpy.init()
    node_left_top = ImageTransformer(camera_position="left_top")
    node_right_top = ImageTransformer(camera_position="right_top")
    node_left_bottom = ImageTransformer(camera_position="left_bottom")
    node_right_bottom = ImageTransformer(camera_position="right_bottom")
    
    Node.run_nodes([node_left_top, node_right_top, node_left_bottom, node_right_bottom])
    rclpy.shutdown()

if __name__ == "__main__":
    main()