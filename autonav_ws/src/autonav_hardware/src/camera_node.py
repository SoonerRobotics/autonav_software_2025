#!/usr/bin/env python3

import rclpy

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

CV_BRIDGE = CvBridge()

#TODO these will not be correct, fix when we actually plug them into the NUC
camera_ids = {
    "front":  0,
    "left" :  1,
    "right":  2,
    "back" :  3
}

# Class to hold our configuration information in
class CameraConfig:
    def __init__(self):
        self.WIDTH = 640
        self.HEIGHT = 480
        self.frameRate = 20


# ROS node to grab frames from a camera and publish them to /autonav/camera for the vision pipeline to take
class CameraNode(Node):
    def __init__(self, direction):
        # direction is a string referring to which side the camera is viewing/facing
        super().__init__("autonav_camera_" + direction)

        self.direction = direction
        self.qos_profile = rclpy.qos.qos_profile_sensor_data #TODO make this more customized, should work for now
        self.config = CameraConfig()

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # make the camera
        self.camera = cv2.VideoCapture(camera_ids[self.direction])
        
        #TODO camera config (white balance, focus, brightness, exposure, etc)

        # publishers
        self.publisher = self.create_publisher(CompressedImage, "/autonav/camera/" + self.direction, self.qos_profile)
        self.publish_timer = self.create_timer(1 / self.config.frameRate, self.publishFrame)

        self.log("Camera " + self.direction + " initialized!")
        self.set_device_state(DeviceState.READY)
    
    def publishFrame(self):
        msg = CompressedImage()

        ret, frame = self.camera.read()

        # if we actually read a frame
        if ret:
            # then transform and publish
            image = cv2.resize(frame, (self.config.WIDTH, self.config.HEIGHT))

            msg = CV_BRIDGE.cv2_to_compressed_imgmsg(image)

            self.publisher.publish(msg)
        else:
            pass # do nothing for now, but maybe TODO add some alert or error logging or something


def main():
    rclpy.init()
    
    # make a camera node for every camera direction
    nodes = [CameraNode(direction) for direction in camera_ids.keys()]

    # https://answers.ros.org/question/377848/spinning-multiple-nodes-across-multiple-threads/
    executor = rclpy.executors.MultiThreadedExecutor()
    
    for node in nodes:
        executor.add_node(node)

    executor.spin()

    # release all the cameras once we're done with them
    [node.camera.release() for node in nodes]

    # and just in case, kill opencv
    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == "__main__":
    main()