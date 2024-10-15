#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorFeedback, MotorControllerDebug, SafetyLights, Conbus
from autonav_shared.types import LogLevel, DeviceState, SystemState
import can


class can_node(Node):
    def __init__self():
        super.__init__("CAN_node")


    def init(self):
        pass


