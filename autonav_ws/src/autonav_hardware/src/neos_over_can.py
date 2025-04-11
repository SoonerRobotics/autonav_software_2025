#!/usr/bin/env python3

import can
import threading
import struct
import time

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorStatistics, ZeroEncoders
from autonav_shared.types import LogLevel, DeviceState, SystemState

ENABLE_API_CLASS = 0 #TODO
ENABLE_API_INDEX = 0 #TODO

NONRIO_HEARTBEAT_API_CLASS = 11
NONRIO_HEARTBEAT_API_INDEX = 2

PERCENT_OUTPUT_API_CLASS = 0
PERCENT_OUTPUT_API_INDEX = 2


class REVMessage():
    def __init__(self, api_class, api_index, device_number, data):
        # see https://docs.google.com/document/d/1ms0ON998f-L-pQZcR1BxyYkEZ_EzRyy0jn7Wsaba6M0/
        self.device_type = 2
        self.manufacturer_code = 5

        self.api_class = api_class
        self.api_index = api_index
        self.device_number = device_number
        self.data = data

    def getArbirtationID(self):
        ret = 0
        ret |= self.device_type << 24
        ret |= self.manufacturer_code << 16
        ret |= self.api_class << 10
        ret |= self.api_index << 6

        return ret #TODO needs to be like 28 bits long or something? idk man
    
    def getMessage(self):
        # https://python-can.readthedocs.io/en/stable/message.html#can.Message
        return can.Message(
            timestamp=time.time(),
            arbitration_id=self.getArbirtationID(),
            # is_extended=True,
            # is_remote_frame=False,
            # is_error_frame=False,
            # is_rx=False,
            # dlc=len(self.data), #TODO
            check=True #TODO
        )


class CANSparkMax():
    def __init__(self, device_id, canbus):
        self.device_id = device_id
        self.canbus = canbus
    
    def enable(self):
        self.canbus.send(REVMessage(
            ENABLE_API_CLASS,
            ENABLE_API_INDEX,
            self.device_id,
            [0x00, 0xFF, 0xFF, 0x00] #TODO FIXME
        ).getMessage())

    def heartbeat(self): #TODO FIXME
        self.canbus.send(REVMessage(
            NONRIO_HEARTBEAT_API_CLASS,
            NONRIO_HEARTBEAT_API_INDEX,
            self.device_id,
            [0x00, 0xFF, 0xFF, 0x00] #TODO FIXME
        ).getMessage())
    
    def set(self, output):
        # self.enable()

        #TODO clamp output or somthing idk
        
        self.canbus.send(REVMessage(
            PERCENT_OUTPUT_API_CLASS,
            PERCENT_OUTPUT_API_INDEX,
            self.device_id,
            [0x0, 0x0, 0x0, 0x0] #TODO
        ).getMessage())


class CanConfig:
    def __init__(self):
        # self.canable_filepath = "/dev/ttyACM0"
        self.canable_filepath = "COM6"


class SparkMAXNode(Node):
    def __init__(self):
        super().__init__("sparkmax_can_node")
        self.write_config(CanConfig())
        self.can = None
        self.motors = []
    
    def init(self):
        # make the CAN object
        # self.can = can.ThreadSafeBus(bustype="slcan", channel=self.config.get("canable_filepath"), bitrate=1_000_000) # FRC CAN runs at 1 Mbit/sec
        self.can = can.ThreadSafeBus(bustype="slcan", channel="/dev/ttyACM0", bitrate=1_000_000) # FRC CAN runs at 1 Mbit/sec
        self.set_device_state(DeviceState.OPERATING)

        # register CAN callback
        self.notifier = can.Notifier(self.can, [self.on_can_received])

        # ROS motor message callback
        self.motorInputSubscriber = self.create_subscription(MotorInput, "/autonav/motor_input", self.on_motor_input_received, 20)

        # Motor objects
        for id in range(1, 9):
            self.motors.append(CANSparkMax(id, self.can))
        
        # Periodic heartbeat to keep motors enabled
        self.heartbeat_timer = self.create_timer(0.05, self.send_heartbeat)
    
    def on_can_received(self, msg):
        #TODO TODO TODO FIXME
        print(f"{hex(msg.arbitration_id)} | {msg.data.hex()}")

    def on_motor_input_received(self, msg):
        # self.can.send() #TODO
        print("Got controller message!")

    def send_heartbeat(self):
        for motor in self.motors:
            motor.heartbeat()


def main():
    rclpy.init()
    can_node = SparkMAXNode()
    try:
        rclpy.spin(can_node)
    except KeyboardInterrupt:
        # shutdown the CAN
        can_node.notifier.shutdown()
        can_node.can.shutdown()

    rclpy.shutdown()

if __name__ == "__main__":
    main()