#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import GPSFeedback
import rclpy

class GpsLoggerNode(Node):
    def __init__(self):
        super().__init__("gps_logger_node")
        self.filename = "./data/gps.csv"
        
    def init(self):
        self.log("Starting GPS logger")

        self.create_subscription(GPSFeedback, "/autonav/gps", self.logging_callback, 1)

        # self.logfile = open(self.filename, "a")

        self.set_device_state(DeviceState.READY)

    def logging_callback(self, msg: GPSFeedback):
        self.log("OPERATING", LogLevel.WARN)
        self.set_device_state(DeviceState.OPERATING)

        lat = msg.latitude
        lon = msg.longitude

        print(f"({lat}, {lon})")

        # self.logfile.write(f"{lat},{lon}\n")


def main():
    rclpy.init()

    node = GpsLoggerNode()
    rclpy.spin(node)

    node.logfile.close()

    rclpy.shutdown()

if __name__ == "__main__":
    main()