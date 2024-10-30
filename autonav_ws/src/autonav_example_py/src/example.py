#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
import rclpy


class ExamplePyConfig:
    def __init__(self):
        self.alpha = 0.5
        self.beta = "Hello"
        self.gamma = 42
        self.delta = True
        self.epsilon = [0.1, 0.2, 0.3]
        self.zeta = ["A", "B", "C"]
        self.eta = {"A": 1, "A": 2, "A": 3}
        self.rid = 2


class ExamplePy(Node):
    def __init__(self):
        super().__init__("autonav_example_py")
        self.write_config(ExamplePyConfig())
        
        self.log("Hello from ExamplePy", LogLevel.DEBUG)
        self.log("Hello from ExamplePy", LogLevel.INFO)
        self.log("Hello from ExamplePy", LogLevel.WARN)
        self.log("Hello from ExamplePy", LogLevel.ERROR)
        self.log("Hello from ExamplePy", LogLevel.FATAL)

    def init(self):
        self.log("Initialized")
        self.set_device_state(DeviceState.READY)

        self.perf_start("example")
        self.perf_stop("example", True)

        self.request_all_configs()

        self.log(f"Beta (CONFIG): {self.config.beta}", LogLevel.DEBUG)
        
def main():
    rclpy.init()
    example = ExamplePy()
    rclpy.spin(example)
    rclpy.shutdown()

if __name__ == "__main__":
    main()