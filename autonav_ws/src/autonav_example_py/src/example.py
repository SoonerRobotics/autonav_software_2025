#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
import rclpy


class Example(Node):
    def __init__(self):
        super().__init__("autonav_example_py")
        
        self.log("Hello from ExamplePy", LogLevel.DEBUG)
        self.log("Hello from ExamplePy", LogLevel.INFO)
        self.log("Hello from ExamplePy", LogLevel.WARN)
        self.log("Hello from ExamplePy", LogLevel.ERROR)
        self.log("Hello from ExamplePy", LogLevel.FATAL)
        
def main():
    rclpy.init()
    example = Example()
    rclpy.spin(example)
    rclpy.shutdown()

if __name__ == "__main__":
    main()