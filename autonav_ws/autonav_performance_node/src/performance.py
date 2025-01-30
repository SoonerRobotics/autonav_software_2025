#!/usr/bin/env python3

import rclpy

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState

import psutil

class PerformanceNode(Node):
    
    def __init__(self):
        self.log("sussy baka")
        


def main(args=None):
    rclpy.init(args=args)

    performance_node = PerformanceNode()

    rclpy.spin(performance_node)
    performance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()