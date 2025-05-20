#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState

class SelfDriveOverwatch(Node):
    def __init__(self):
        super().__init__("self_drive_overwatch")

    def init(self):
        self.set_device_state(DeviceState.READY)

    # def on_system_state_updated(self, old, new):
    #     if new == SystemState.AUTONOMOUS and self.device_states.get(self.get_name()) == DeviceState.READY:
    #         self.set_device_state(DeviceState.OPERATING)

    #     if new != SystemState.AUTONOMOUS and self.device_states.get(self.get_name()) == DeviceState.OPERATING:
    #         self.set_device_state(DeviceState.READY)

 
def main():
    rclpy.init()
    Node.run_node(SelfDriveOverwatch())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
