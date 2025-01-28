#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import json
import cv2
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import MapMetaData, OccupancyGrid
from autonav_shared.types import DeviceState
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose

from autonav_shared.node import Node
from nav_msgs.msg import OccupancyGrid


g_bridge = CvBridge()
g_mapData = MapMetaData()
g_mapData.width = 200  #width for four grids
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_vision_combiner")

    def init(self):
        self.grid_leftFront = None
        self.grid_rightFront = None
        self.grid_leftBack = None
        self.grid_rightBack = None

        # Subscribe to four camera grid topics
        self.grid_leftFront_subscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw/leftFront", self.grid_received_leftFront, 1)
        self.grid_rightFront_subscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw/rightFront", self.grid_received_rightFront, 1)
        self.grid_leftBack_subscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw/leftBack", self.grid_received_leftBack, 1)
        self.grid_rightBack_subscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw/rightBack", self.grid_received_rightBack, 1)

        self.combined_grid_publisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/combined", 1)
        self.combined_grid_image_publisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/combined/image", 1)

        self.set_device_state(DeviceState.OPERATING)

    def grid_received_leftFront(self, msg):
        self.grid_leftFront = msg
        self.try_combine_grids()

    def grid_received_rightFront(self, msg):
        self.grid_rightFront = msg
        self.try_combine_grids()

    def grid_received_leftBack(self, msg):
        self.grid_leftBack = msg
        self.try_combine_grids()

    def grid_received_rightBack(self, msg):
        self.grid_rightBack = msg
        self.try_combine_grids()

    def try_combine_grids(self):
        #all four grids are received before combining
        if self.grid_leftFront is None or self.grid_rightFront is None or self.grid_leftBack is None or self.grid_rightBack is None:
            return

        #combined grid with the dimensions to accommodate four grids
        combined_grid = OccupancyGrid()
        combined_grid.header = self.grid_leftFront.header
        combined_grid.info = self.grid_leftFront.info
        combined_grid.info.width = 160  # Double width
        combined_grid.info.height = 100  
        combined_grid.data = [0] * (combined_grid.info.width * combined_grid.info.height)

        #Combine grids
        for y in range(100):  # Assuming 100 rows of grid data
            for x in range(80):  #leftFront side
                combined_grid.data[y * 160 + x] = self.grid_leftFront.data[y * 80 + x]
            for x in range(80, 160):  #rightFront side
                combined_grid.data[y * 160 + x] = self.grid_rightFront.data[y * 80 + (x - 80)]
            for x in range(160, 240):  #leftBack grids
                combined_grid.data[y * 160 + x] = self.grid_leftBack.data[y * 80 + (x - 160)]
            for x in range(240, 320):  #rightBack grids
                combined_grid.data[y * 160 + x] = self.grid_rightBack.data[y * 80 + (x - 240)]

        #Publish combined grid
        self.grid_leftFront = None
        self.grid_rightFront = None
        self.grid_leftBack = None
        self.grid_rightBack = None
        self.combined_grid_publisher.publish(combined_grid)

        #Publish combined grid as image
        preview_image = np.zeros((100, 160), dtype=np.uint8)
        for i in range(100):
            for j in range(160):
                preview_image[i, j] = 0 if combined_grid.data[i * 160 + j] <= 10 else 255
        preview_image = cv2.cvtColor(preview_image, cv2.COLOR_GRAY2RGB)
        preview_image = cv2.resize(preview_image, (640, 320))
      

        # Draw a grid on the image to indicate the individual sections
        for i in range(100):
            cv2.line(preview_image, (0, i * 3), (640, i * 3), (85, 85, 85), 1)
            cv2.line(preview_image, (i * 4, 0), (i * 4, 320), (85, 85, 85), 1)
            cv2.line()

        compressed_image = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        self.combined_grid_image_publisher.publish(compressed_image)

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return {}


def main():
    rclpy.init()
    node = ImageCombiner()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()