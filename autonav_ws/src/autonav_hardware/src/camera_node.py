#!/usr/bin/env python3

import rclpy

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from sensor_msgs.msg import CompressedImage

import cv2
import threading
import time
from cv_bridge import CvBridge

CV_BRIDGES = {
    "left" : CvBridge(),
    "front": CvBridge(),
    "right": CvBridge(),
    "back" : CvBridge()
}

#TODO these will not be correct, fix when we actually plug them into the NUC
camera_ids = {
    "front":  0,
    # "left" :  2,
    # "right":  4,
    # "back" :  1
}

camera_settings = {
    "front": {
        "flip": True
    }
}

# Class to hold our configuration information in
class CameraConfig:
    def __init__(self):
        self.width = 640
        self.height = 480
        self.frame_rate = 15


# ROS node to grab frames from a camera and publish them to /autonav/camera for the vision pipeline to take
class CameraNode(Node):
    def __init__(self, direction):
        # direction is a string referring to which side the camera is viewing/facing
        super().__init__("autonav_camera_" + direction)
        self.config = CameraConfig()
        self.direction = direction
        # self.qos_profile = rclpy.qos.qos_profile_sensor_data #TODO make this more customized, should work for now

    def init(self):
        # make the camera
        
        #TODO camera config (white balance, focus, brightness, exposure, etc)
        self.camera = None

        # publishers
        self.publisher = self.create_publisher(CompressedImage, "/autonav/camera/" + self.direction, 20)
        # self.publish_timer = self.create_timer(1 / self.config.frame_rate, self.publishFrame)
        self.next_pub_time = 0
        self.big_worker = threading.Thread(target=self.publishFrame)
        self.big_worker.daemon = True
        self.big_worker.start()

        self.log("Camera " + self.direction + " initialized!")
        self.set_device_state(DeviceState.READY)

    def apply_config(self, config):
        self.config.width = config["width"]
        self.config.height = config["height"]
        self.config.frame_rate = config["frame_rate"]
    
    def publishFrame(self):
        while rclpy.ok():
            if self.camera is None or not self.camera.isOpened():
                self.camera = cv2.VideoCapture(camera_ids[self.direction])

                # Set their resolution to the same
                # set their frame rate to the same
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
                # self.camera.set(cv2.CAP_PROP_FPS, self.config.frame_rate)
                # self.camera.set(cv2.CAP_PROP_BITRATE, 5)
                self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

            # check if the camera is open
            if not self.camera.isOpened():
                # self.log("Camera " + self.direction + " failed to open!", LogLevel.ERROR)
                continue

            # check if we should publish
            if time.time() > self.next_pub_time:
                self.next_pub_time = time.time() + 1 / self.config.frame_rate

                # get the frame
                ret, frame = self.camera.read()

                # check if we got a frame
                if not ret:
                    # self.log("Camera " + self.direction + " failed to read frame!", LogLevel.ERROR)
                    time.sleep(max(self.next_pub_time - time.time(), 0))
                    continue

                # Check if we need to flip the frame
                # if camera_settings[self.direction]["flip"]:
                    # flip the frame vertically
                frame = cv2.flip(frame, 0)
                frame = cv2.flip(frame, 1)

                # self.log("big camera capture")
                # self.get_logger().info("big camera")

                # convert the frame to a CompressedImage
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = CV_BRIDGES[self.direction].cv2_to_compressed_imgmsg(frame).data
                # self.log("Camera " + self.direction + " publishing frame!", LogLevel.DEBUG)

                # publish the image
                self.publisher.publish(msg)
                time.sleep(max(0, self.next_pub_time - time.time()))


def main():
    rclpy.init()
    
    # make a camera node for every camera direction
    # nodes = [CameraNode(direction) for direction in camera_ids.keys()]

    # https://answers.ros.org/question/377848/spinning-multiple-nodes-across-multiple-threads/
    # executor = rclpy.executors.MultiThreadedExecutor()

    # nodes = [
        # CameraNode("left"),
        # CameraNode("front"),
        # CameraNode("right"),
        # CameraNode("back"),
    # ]
    
    # for node in nodes:
        # executor.add_node(node)

    # executor.spin()

    # release all the cameras once we're done with them
    # [node.camera.release() for node in nodes]

    nod = CameraNode("front")

    rclpy.spin(nod)

    # and just in case, kill opencv
    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
