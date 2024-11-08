#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorFeedback, MotorControllerDebug, SafetyLights, Conbus
from autonav_shared.types import LogLevel, DeviceState, SystemState
import can
import threading
import struct
from ctypes import Structure, c_bool, c_uint8

arbitration_ids = {
    "EStop": 0,
    "MobilityStop": 1,
    "MobilityStart": 9,
    "MotorsCommand": 10,
    "SafetyLightsCommand": 13,
    "OdometryFeedback": 14,
    "ObjectDetection": 20,
    "LinearPIDStatistics": 50,
    "AngularPIDStatistics": 51,
    "MotorStatisticsFrontMotors": 52,
    "MotorStatisticsBackMotors": 53,
    "ConbusLowerBound": 1000,
    "ConbusUpperBound": 1400
}

class SafetyLightsPacket(Structure):
    _fields_ = [
        ("autonomous", c_bool, 1),
        ("mode", c_uint8, 7),
        ("brightness", c_uint8, 8),
        ("red", c_uint8, 8),
        ("green", c_uint8, 8),
        ("blue", c_uint8, 8),
        ("blink_period", c_uint8, 8)
    ]

class CanConfig:
    def __init__(self):
        self.odom_feedback_scaler = 10000


class can_node(Node):
    def __init__(self):
        super.__init__("CAN_node")
        self.write_config(CanConfig)

        # can
        self.can = None

        # safety lights
        self.safetyLightsSubscriber = self.create_subscription(
            SafetyLights,
            "autonav/safety_lights",
            self.on_safety_lights_received(),
            20
        )

        # motor messages
        self.motorInputSubscriber = self.create_subscription(
            MotorInput,
            "autonav/motor_feedback",
            self.on_motor_input_received(),
            20
        )
        self.motorFeedbackPublisher = self.create_publisher(
            MotorFeedback,
            "/autonav/MotorFeedback", 
            20
        )

        # conbus
        self.conbusSubscriber = self.create_subscription(
            Conbus,
            "/autonav/conbus/instruction", 
            self.on_conbus_received(), 
            20
        )
        self.conbusPublisher = self.create_publisher(
            Conbus, 
            "/autonav/conbus/data", 
            20
        )


    def init(self):
        # can threading
        self.canTimer = self.create_timer(0.5, self.can_worker)
        self.canReadThread = threading.Thread(target=self.can_thread_worker)
        self.canReadThread.daemon = True


    def can_worker(self):
        try:
            with open("/dev/ttyACM0", "r") as f:
                pass

            if self.can is not None:
                return

            self.can = can.ThreadSafeBus(
                bustype="slcan", channel="/dev/ttyACM0", bitrate=100000)
            self.set_device_state(DeviceState.OPERATING)
        except:
            if self.can is not None:
                self.can = None

            if self.device_state != DeviceState.WARMING:
                self.set_device_state(DeviceState.WARMING)


    def can_thread_worker(self):
        while rclpy.ok():
            if self.device_state != DeviceState.READY and self.device_state != DeviceState.OPERATING:
                continue
            if self.can is not None:
                try:
                    msg = self.can.recv(timeout=0.01)
                    if msg is not None:
                        self.onCanMessageReceived(msg)
                except can.CanError:
                    pass
    

    def onCanMessageReceived(self, msg):
        arbitration_id = msg.arbitration_id
        if arbitration_id == arbitration_ids["Estop"]:
            self.set_mobility(False)

        if arbitration_id == arbitration_ids["MobilityStart"]:
            self.set_mobility(True)
        
        if arbitration_id == arbitration_ids["MobilityStop"]:
            self.set_mobility(False)

        if arbitration_id == arbitration_ids["OdometryFeedback"]:
            self.publish_odom_feedback(msg)

        if arbitration_id == arbitration_ids["LinearPIDStatistics"]:
            pass

        if arbitration_id >= arbitration_ids["ConbusLowerBound"] and arbitration_id < arbitration_ids["ConbusUpperBound"]:
            self.publish_conbus(msg)


    def publish_odom_feedback(self, msg):
        delta_x, delta_y, delta_theta = struct.unpack('hhh', msg.data)
        motor_feedback_msg = MotorFeedback()
        motor_feedback_msg.delta_x = delta_x / self.config.odom_feedback_scaler
        motor_feedback_msg.delta_y = delta_y / self.config.odom_feedback_scaler
        motor_feedback_msg.delta_theta = delta_theta / self.config.odom_feedback_scaler

        self.motorFeedbackPublisher.publish(motor_feedback_msg)


    def publish_conbus(self, msg):
        conbus = Conbus()
        conbus.id = msg.arbitration_id
        conbus.data = msg.data
        self.conbusPublisher.publish(conbus)


    # subscriber callbacks
    def on_safety_lights_received(self, msg:SafetyLights):
        safety_lights_packet = SafetyLightsPacket()
        safety_lights_packet.autonomous = msg.autonomous
        safety_lights_packet.mode = msg.mode
        safety_lights_packet.red = msg.red
        safety_lights_packet.green = msg.green
        safety_lights_packet.blue = msg.blue
        safety_lights_packet.blink_period = msg.blink_period

        data = bytes(safety_lights_packet)
        can_msg = can.Message(arbitration_id=arbitration_ids["SafetyLightsCommand"], data=data)

        try:
            self.can.send(can_msg)
        except can.CanError:
            pass

    def on_motor_input_received(self, msg:MotorInput):
        if self.device_state != DeviceState.OPERATING:
            return
        data = struct.pack("hhh", int(msg.forward_velocity * 10000), int(msg.sideways_velocity * 10000), int(msg.angular_velocity * 10000))
        can_msg = can.message(arbitration_id = arbitration_ids["MotorsCommand"], data = data)
        try:
            self.can.send(can_msg)
        except can.CanError:
            pass


    def on_conbus_received(self, msg:Conbus):
        if self.device_state != DeviceState.OPERATING:
            return
        try:
            data = bytes(msg.data)
            arbitration_id = msg.id

            can_msg = can.message(arbitration_id = arbitration_id, data = data)
            
            try:
                self.can.send(can_msg)
            except can.CanError:
                pass
        except:
            pass
    


