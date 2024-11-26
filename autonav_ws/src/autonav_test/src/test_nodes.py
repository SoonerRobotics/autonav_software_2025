#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import rclpy, cv2
import numpy as np

bridge = CvBridge() 

class CameraLoggerNode(Node):
    def __init__(self, id: str):
        super().__init__("camera_logger_node_" + id)
        self.id = id
        self.filename = "./data/" + self.id + ".mp4"
        self.frame = 0

    def init(self):
        self.log("Starting camera logger: " + self.id)

        self.create_subscription(CompressedImage, "/autonav/camera/" + self.id, self.camera_callback, 1)

        self.video_writer = cv2.VideoWriter(self.filename, cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (640, 480)) #TODO

        self.set_device_state(DeviceState.WARMING)

    def camera_callback(self, msg):
        # self.set_device_state(DeviceState.OPERATING)

        img = bridge.compressed_imgmsg_to_cv2(msg)

        if self.frame < 100:
            self.video_writer.write(img)
        elif self.video_writer.isOpened():
            self.video_writer.release()
            self.log("Camera logger " + self.id + " is done!", LogLevel.WARN)
        
        self.frame += 1

def main():
    rclpy.init()

    nodes = []

    for id in ["front", "left", "right", "back"]:
        nodes.append(CameraLoggerNode(id))

    # https://github.com/SoonerRobotics/autonav_software_2025/blob/feat/camera/autonav_ws/src/autonav_hardware/src/camera_node.py
    executor = rclpy.executors.MultiThreadedExecutor()
    
    for node in nodes:
        executor.add_node(node)

    executor.spin()

    cv2.destroyAllWindows() # just in case, you never know

    rclpy.shutdown()

if __name__ == "__main__":
    main()