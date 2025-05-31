#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorFeedback, SafetyLights, Ultrasonic, Conbus, CanStats
from autonav_msgs.msg import LinearPIDStatistics, AngularPIDStatistics, MotorStatistics, ZeroEncoders
from autonav_shared.types import LogLevel, DeviceState, SystemState
import can
import threading
import struct
import time
from ctypes import Structure, c_uint8, c_bool


CAN_PATH = "/dev/autonav-can-scr"
CAN_SPEED = 100_000


arbitration_ids = {
    "EStop": 0,
    "MobilityStop": 1,
    "MobilityStart": 9,
    "MotorsCommand": 10,
    "SafetyLightsCommand": 13,
    "ObjectDetection": 20,
    "ConbusLowerBound": 1000,
    "ConbusUpperBound": 1400
}


class SafetyLightsPacket(Structure):
    _fields_ = [
        ("autonomous", c_bool, 1),
        ("eco", c_uint8, 1),
        ("mode", c_uint8, 6),
        ("brightness", c_uint8, 8),
        ("red", c_uint8, 8),
        ("green", c_uint8, 8),
        ("blue", c_uint8, 8),
        ("blink_period", c_uint8, 8)
    ]

class CanNode(Node):
    def __init__(self):
        super().__init__("autonav_can")
        self.can_stats_record = CanStats()
        self.can = None
        self.send_rainbow = 0

        # safety lights
        self.safetyLightsSubscriber = self.create_subscription(
            SafetyLights,
            "/autonav/safety_lights",
            self.on_safety_lights_received,
            20
        )
        
        # object detection
        self.objectDetectionPublisher = self.create_publisher(
            Ultrasonic,
            "/autonav/ultrasonic",
            20
        )

        # conbus
        self.conbusSubscriber = self.create_subscription(
            Conbus,
            "/autonav/conbus/instruction", 
            self.on_conbus_received, 
            20
        )

        self.conbusPublisher = self.create_publisher(
            Conbus, 
            "/autonav/conbus/data", 
            20
        )

        # CAN utilization stats
        self.can_stats_publisher = self.create_publisher(
            CanStats,
            "/autonav/can_stats",
            20
        )


    def init(self):
        # can threading
        self.canTimer = self.create_timer(0.5, self.can_worker)
        self.canStatsTimer = self.create_timer(10, self.publish_can_stats)
        self.canReadThread = threading.Thread(target=self.can_thread_worker)
        self.canReadThread.daemon = True
        self.canReadThread.start()

    def send_startup_can(self):
        time.sleep(1)
        safety_lights_packet = SafetyLightsPacket()
        safety_lights_packet.autonomous = True
        safety_lights_packet.eco = False
        safety_lights_packet.mode = 1
        safety_lights_packet.brightness = 255
        safety_lights_packet.red = 0
        safety_lights_packet.green = 0
        safety_lights_packet.blue = 255
        safety_lights_packet.blink_period = 10
        data = bytes(safety_lights_packet)
        can_msg = can.Message(arbitration_id=arbitration_ids["SafetyLightsCommand"], data=data)
        self.can.send(can_msg)
        self.send_rainbow = time.time() + 5

    def can_worker(self):
        try:
            with open(CAN_PATH, "r") as f:
                pass

            if self.can is not None:
                if self.send_rainbow > time.time() and self.send_rainbow != 0:
                    safety_lights_packet = SafetyLightsPacket()
                    safety_lights_packet.autonomous = False
                    safety_lights_packet.eco = False
                    safety_lights_packet.mode = 3
                    safety_lights_packet.brightness = 255
                    safety_lights_packet.red = 255
                    safety_lights_packet.green = 255
                    safety_lights_packet.blue = 255
                    safety_lights_packet.blink_period = 50
                    data = bytes(safety_lights_packet)
                    can_msg = can.Message(arbitration_id=arbitration_ids["SafetyLightsCommand"], data=data)
                    self.can.send(can_msg)
                    self.send_rainbow = 0
                return

            self.log("CAN device found at " + CAN_PATH, LogLevel.INFO)
            self.can = can.ThreadSafeBus(
                bustype="slcan", channel=CAN_PATH, bitrate=CAN_SPEED)
            self.set_device_state(DeviceState.OPERATING)
            self.send_startup_can()
        except:
            self.log("CAN device not found at " + CAN_PATH, LogLevel.ERROR)
            if self.can is not None:
                self.can = None

            if self.get_device_state() != DeviceState.READY:
                self.set_device_state(DeviceState.READY)


    def can_thread_worker(self):
        while rclpy.ok():
            if self.get_device_state() != DeviceState.OPERATING:
                continue
            
            if self.can is not None:
                try:
                    msg = self.can.recv()
                    if msg is not None:
                        self.onCanMessageReceived(msg)
                except Exception as e:
                    self.log(f"Received erroneous CAN message from hardware {e}", LogLevel.ERROR)
    

    def onCanMessageReceived(self, msg):
        self.can_stats_record.rx = self.can_stats_record.rx + 1
        arbitration_id = msg.arbitration_id

        self.log(f"Received CAN message with id: {arbitration_id}", LogLevel.DEBUG)

        if arbitration_id == arbitration_ids["EStop"]:
            self.set_mobility(False)
            self.set_system_state(SystemState.DISABLED)

        elif arbitration_id == arbitration_ids["MobilityStart"]:
            self.set_mobility(True)
        
        elif arbitration_id == arbitration_ids["MobilityStop"]:
            self.set_mobility(False)

        elif arbitration_id == arbitration_ids["ObjectDetection"]:
            self.publish_object_detection(msg)

        elif arbitration_id >= arbitration_ids["ConbusLowerBound"] and arbitration_id < arbitration_ids["ConbusUpperBound"]:
            self.publish_conbus(msg)

        else:
            self.log(f"Received CAN message with invalid id: {arbitration_id}, data: {msg.data}", LogLevel.ERROR)

    def publish_object_detection(self, msg):
        ultrasonic_msg = Ultrasonic()
        data = bytes(msg.data)
        ultrasonic_msg.id = data[0]
        ultrasonic_msg.distance = (data[1] << 8) | data[2]
        self.objectDetectionPublisher.publish(ultrasonic_msg)

    def publish_conbus(self, msg):
        conbus = Conbus()
        conbus.id = msg.arbitration_id
        conbus.data = msg.data
        self.conbusPublisher.publish(conbus)

    # subscriber callbacks
    def on_safety_lights_received(self, msg:SafetyLights):
        self.can_stats_record.tx = self.can_stats_record.tx + 1
        if self.get_device_state() != DeviceState.OPERATING:
            return

        self.send_rainbow = 0
        safety_lights_packet = SafetyLightsPacket()
        safety_lights_packet.autonomous = msg.mode == 1
        safety_lights_packet.eco = False
        safety_lights_packet.mode = msg.mode
        safety_lights_packet.brightness = msg.brightness
        safety_lights_packet.red = msg.red
        safety_lights_packet.green = msg.green
        safety_lights_packet.blue = msg.blue
        safety_lights_packet.blink_period = msg.blink_period
        data = bytes(safety_lights_packet)
        can_msg = can.Message(arbitration_id=arbitration_ids["SafetyLightsCommand"], data=data)

        self.log(f"Sending safety lights command: {safety_lights_packet}", LogLevel.DEBUG)
        
        try:
            self.can.send(can_msg)
        except AttributeError:
            self.log("Error writing to CAN", LogLevel.ERROR)
            pass # means the CAN object hasn't been created yet
        except can.CanError:
            self.log("2 Error writing to CAN", LogLevel.ERROR)
            pass

    def on_conbus_received(self, msg:Conbus):
        self.can_stats_record.tx = self.can_stats_record.tx + 1
        if self.get_device_state() != DeviceState.OPERATING:
            return

        data = bytes(msg.data)
        arbitration_id = msg.id
        can_msg = can.Message(arbitration_id = arbitration_id, data = data)
        
        try:
            self.can.send(can_msg)
        except AttributeError:
            pass
        except can.CanError:
            pass    

    def publish_can_stats(self):
        self.can_stats_publisher.publish(self.can_stats_record)

def main():
    
    rclpy.init()
    can_node = CanNode()
    rclpy.spin(can_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

