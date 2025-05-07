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
        super().__init__("example_py")

        # This will initialize our default config
        self.config = ExamplePyConfig()

        # DO NOT USE CONFIG VARIABLES IN THE CONSTRUCTOR
        # ^ the config may be updated by the commander. Do all your initialization that requires the config in init()

    def apply_config(self, config: dict):
        # TODO: Figure out if python has a better way to do this
        # This is called BEFORE on_config_update
        # The point of this is to convert from the generic json dict to our config object
        self.config.alpha = config["alpha"]
        self.config.beta = config["beta"]
        self.config.gamma = config["gamma"]
        self.config.delta = config["delta"]
        self.config.epsilon = config["epsilon"]
        self.config.zeta = config["zeta"]
        self.config.eta = config["eta"]
        self.config.rid = config["rid"]

    def init(self):
        # Device state at this point will be WARMING

        self.log("Initialized")
        self.set_device_state(DeviceState.READY)

        self.perf_start("example")
        self.perf_stop("example", True)

    def on_config_update(self, old_cfg, new_cfg):
        self.log(f"Config updated from {old_cfg} to {new_cfg}")

    def on_system_state_updated(self, old, new):
        self.log(f"System state changed from {old} to {new}")

    def on_mobility_updated(self, old, new):
        self.log(f"Mobility state changed from {old} to {new}")
        
def main():
    rclpy.init()
    example = ExamplePy()
    rclpy.spin(example)
    rclpy.shutdown()

if __name__ == "__main__":
    main()