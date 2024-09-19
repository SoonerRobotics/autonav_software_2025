#!/usr/bin/env python3

from autonav_shared.node import Node
import rclpy


class Example(Node):
    def __init__(self):
        super().__init__("autonav_example_py")

def main():
    rclpy.init()
    example = Example()
    rclpy.spin(example)
    rclpy.shutdown()

if __name__ == "__main__":
    main()