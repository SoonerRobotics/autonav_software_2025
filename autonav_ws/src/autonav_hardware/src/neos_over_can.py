#!/usr/bin/env python3

import can
import threading
from struct import pack, unpack
import time

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorStatistics, ZeroEncoders
from autonav_shared.types import LogLevel, DeviceState, SystemState

ENABLE_API_CLASS = 11
ENABLE_API_INDEX = 0
# ENABLE_API_INDEX = 10 # system resume according to FRC docs???

NONRIO_HEARTBEAT_API_CLASS = 11
NONRIO_HEARTBEAT_API_INDEX = 2

PERCENT_OUTPUT_API_CLASS = 0
PERCENT_OUTPUT_API_INDEX = 2

ENCODER_API_CLASS = 6 # periodic status frame 2 has encoder position data
ENCODER_API_INDEX = 2

POSITION_API_CLASS = 3
POSITION_API_INDEX = 2

PARAMETER_API_CLASS = 48
PARAMETER_API_INDEX = 0

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
        ret |= self.device_number

        return ret
    
    def getMessage(self):
        # https://python-can.readthedocs.io/en/stable/message.html#can.Message
        return can.Message(
            timestamp=time.time(),
            arbitration_id=self.getArbirtationID(),
            data=self.data,
            check=True #TODO we can remove this eventually I think
        )


class CANSparkMax():
    def __init__(self, device_id, canbus):
        self.device_id = device_id
        self.canbus = canbus

        self.position = 0 # rotations, TODO FIXME
        self.velocity = 0 # RPM
    
    def enable(self):
        self.canbus.send(REVMessage(
            ENABLE_API_CLASS,
            ENABLE_API_INDEX,
            self.device_id,
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF] #TODO FIXME not sure what's supposed to go here
        ).getMessage())

    def heartbeat(self): #TODO FIXME
        self.enable()

        self.canbus.send(REVMessage(
            NONRIO_HEARTBEAT_API_CLASS,
            NONRIO_HEARTBEAT_API_INDEX,
            self.device_id,
            [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00] #TODO FIXME are these supposed to individually enable motors? should this bytearray enable all of them? should we only enable if we are in manual?
        ).getMessage())

    def configure(self):
        #TODO FIXME set like, parameters and stuff here (PIDs, update rates, etc)
        # encoder configuration (periodic status frame 02?) probably, from REV hardware client:
        #0x02050401  DLC=4  Data=[04, 00, 04, 00]
        #0x02050441  DLC=5  Data=[00, 04, 00, 27, 00]

        # enable periodic status frame 2 with an update rate of 30ms
        self.canbus.send(REVMessage(
            ENCODER_API_CLASS,
            ENCODER_API_INDEX,
            self.device_id,
            [0x1D, 0x00] # update rate, in milliseconds, in reverse order
        ).getMessage())

        # self.canbus.send(REVMessage(
            #TODO config PID values
        # ).getMessage())
    
    def set(self, output):
        # self.enable() #TODO we should only send this if we are in fact enabled and in manual mode or whatever

        # clamp output TODO we should set the kMaxOutput values for each sparkmax too somewhere at some point
        if output < -1:
            output = -1
        elif 1 < output:
            output = 1

        data_array = bytearray(pack('<f', output)) # NEOs expect little-endian format
        
        # need to append 4*4=16 trailing bytes I think (like, 4 different pairs of hex digits, so 2*4*2 or something?)
        for i in range(4):
            data_array.append(0x00) #FIXME when you print it something looks off about the bytearray (it has an '=' in it somewhere) but it works so

        self.canbus.send(REVMessage(
            PERCENT_OUTPUT_API_CLASS,
            PERCENT_OUTPUT_API_INDEX,
            self.device_id,
            data_array,
        ).getMessage())
    
    def setPosition(self, pos):
        data_array = bytearray(pack('<f', pos)) # NEOs expect little-endian format
        
        # need to append 4*4=16 trailing bytes I think (like, 4 different pairs of hex digits, so 2*4*2 or something?)
        for i in range(4):
            data_array.append(0x00) #FIXME when you print it something looks off about the bytearray (it has an '=' in it somewhere) but it works so

        self.canbus.send(REVMessage(
            POSITION_API_CLASS,
            POSITION_API_INDEX,
            self.device_id,
            data_array,
        ).getMessage())
    
    def getEncoderArbID(self):
        return REVMessage(
            ENCODER_API_CLASS,
            ENCODER_API_INDEX,
            self.device_id,
            []
        ).getMessage().arbitration_id


#FIXME CanConfig doesn't do anything right now
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
        self.hasConfigured = False
    
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
    
    def on_can_received(self, msg: can.Message):
        # ignore the boring ones. might be worth looking into actually making them not update as fast, or hardware filtering that python-can mentions, but we're not coding for the future right now
        match(msg.arbitration_id):
            case 0x205bc01:
                return
            case 0x205b801:
                return
            case 0x205b841:
                return
            case 0x2051801:
                return
            
        # print(f"{hex(msg.arbitration_id)} | {msg.data.hex()}")

        for motor in self.motors:
            if motor.getEncoderArbID() == msg.arbitration_id:
                print(unpack('<ff', msg.data)[0]) # units are rotation, don't care about the last 4 bytes


    def on_motor_input_received(self, msg):
        # self.can.send() #TODO
        print("Got controller message!")

    def send_heartbeat(self):
        for motor in self.motors:
            motor.heartbeat()
        
        if not self.hasConfigured:
            for motor in self.motors:
                motor.configure()
            self.hasConfigured = True


def main():
    rclpy.init()
    can_node = SparkMAXNode()
    try:
        rclpy.spin(can_node)
    except KeyboardInterrupt:
        # shutdown the CAN
        can_node.notifier.stop()
        can_node.can.shutdown()

    rclpy.shutdown()

if __name__ == "__main__":
    main()