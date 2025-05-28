#!/usr/bin/env python3

import rclpy

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import HardwarePerformance

import psutil

class PerformanceConfig():
    def __init__(self):
        self.CELSIUS = False

class PerformanceNode(Node):
    def __init__(self):
        super().__init__("autonav_hardware_performance")

        # Initiliaze config
        self.config = PerformanceConfig()
                
    def init(self):
        # Publisher
        self.performance_publisher = self.create_publisher(HardwarePerformance, "/autonav/hardware_performance", 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    
    def timer_callback(self):
        msg = HardwarePerformance()
        msg.cpu = psutil.cpu_percent()
        msg.memory = psutil.virtual_memory().percent
        msg.temp = str(psutil.sensors_temperatures(self.config.CELSIUS))
        
        self.log(f"{msg.cpu}, {msg.memoru}, {msg.temp}", LogLevel.INFO)

        self.performance_publisher.publish(msg)


def main():
    rclpy.init()

    performance_node = PerformanceNode()

    rclpy.spin(performance_node)
    performance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()