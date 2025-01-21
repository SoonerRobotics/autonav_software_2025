#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorFeedback, SafetyLights, Conbus
from autonav_msgs.msg import LinearPIDStatistics, AngularPIDStatistics, MotorStatisticsFrontMotors, MotorStatisticsBackMotors
from autonav_shared.types import LogLevel, DeviceState, SystemState
import can
import threading
import struct
from ctypes import Structure, c_bool, c_uint8
import binascii

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
        ("mode", c_uint8),
        ("brightness", c_uint8),
        ("red", c_uint8),
        ("green", c_uint8),
        ("blue", c_uint8),
        ("blink_period", c_uint8)
    ]

class CanConfig:
    def __init__(self):
        self.canable_filepath = "/dev/ttyACM0"
        self.odom_feedback_scaler = 10000
        self.linear_pid_scaling_factor = 1000
        self.angular_pid_scaling_factor = 1000


class CanNode(Node):
    def __init__(self):
        super().__init__("CAN_node")
        self.write_config(CanConfig())

        # can
        self.can = None

        # safety lights
        self.safetyLightsSubscriber = self.create_subscription(
            SafetyLights,
            "/autonav/safety_lights",
            self.on_safety_lights_received,
            20
        )

        # motor messages
        self.motorInputSubscriber = self.create_subscription(
            MotorInput,
            "/autonav/motor_input",
            self.on_motor_input_received,
            20
        )

        self.motorFeedbackPublisher = self.create_publisher(
            MotorFeedback,
            "/autonav/motor_feedback", 
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

        # motor statistics and PID tuning
        self.linearPIDStatisticsPublisher = self.create_publisher(
            LinearPIDStatistics,
            "/autonav/linear_pid_statistics",
            20
        )
        self.angularPIDStatisticsPublisher = self.create_publisher(
            AngularPIDStatistics,
            "/autonav/angular_pid_statistics",
            20
        )
        self.motorStatisticsFrontMotorsPublisher = self.create_publisher(
            MotorStatisticsFrontMotors,
            "/autonav/motor_statistics_front_motors",
            20
        )
        self.motorStatisticsBackMotorsPublisher = self.create_publisher(
            MotorStatisticsBackMotors,
            "/autonav/motor_statistics_back_motors",
            20
        )


    def init(self):
        # can threading
        self.log("initializing CAN node", LogLevel.DEBUG)
        self.canTimer = self.create_timer(0.5, self.can_worker)
        self.canReadThread = threading.Thread(target=self.can_thread_worker)
        self.canReadThread.daemon = True
        self.canReadThread.start()


    def can_worker(self):
        try:
            with open(self.config.get("canable_filepath"), "r") as f:
                pass

            if self.can is not None:
                return

            self.can = can.ThreadSafeBus(
                bustype="slcan", channel=self.config.get("canable_filepath"), bitrate=100000)
            self.set_device_state(DeviceState.OPERATING)
        except:
            if self.can is not None:
                self.can = None

            if self.get_device_state() != DeviceState.WARMING:
                self.set_device_state(DeviceState.WARMING)


    def can_thread_worker(self):
        self.log("can thread worker working", LogLevel.DEBUG)
        while rclpy.ok():
            if self.get_device_state() != DeviceState.READY and self.get_device_state() != DeviceState.OPERATING:
                continue
            if self.can is not None:
                try:
                    msg = self.can.recv(timeout=0.01)
                    if msg is not None:
                        self.onCanMessageReceived(msg)
                except can.CanError:
                    pass
    

    def onCanMessageReceived(self, msg):
        self.log("CAN message received", LogLevel.DEBUG)
        arbitration_id = msg.arbitration_id
        self.log(f"{arbitration_id}", LogLevel.DEBUG)
        if arbitration_id == arbitration_ids["EStop"]:
            self.log("EStop", LogLevel.DEBUG)
            self.set_mobility(False)

        if arbitration_id == arbitration_ids["MobilityStart"]:
            self.log("Mobility Start", LogLevel.DEBUG)
            self.set_mobility(True)
        
        if arbitration_id == arbitration_ids["MobilityStop"]:
            self.log("Mobility Stop", LogLevel.DEBUG)
            self.set_mobility(False)

        if arbitration_id == arbitration_ids["OdometryFeedback"]:
            self.log("Odom Feedback", LogLevel.DEBUG)
            self.publish_odom_feedback(msg)

        if arbitration_id == arbitration_ids["LinearPIDStatistics"]:
            self.log("Linear PID Stats", LogLevel.DEBUG)
            self.publish_linear_pid_statistics(msg)
    
        if arbitration_id == arbitration_ids["AngularPIDStatistics"]:
            self.log("Angular PID Stats", LogLevel.DEBUG)
            self.publish_angular_pid_statistics(msg)

        if arbitration_id == arbitration_ids["MotorStatisticsFrontMotors"]:
            self.log("Motor Statistics Front Motors", LogLevel.DEBUG)
            self.publish_motor_statistics_front_motors(msg)
        
        if arbitration_id == arbitration_ids["MotorStatisticsBackMotors"]:
            self.log("Motor Statistics Back Motors", LogLevel.DEBUG)
            self.publish_motor_statistics_back_motors(msg)

        if arbitration_id >= arbitration_ids["ConbusLowerBound"] and arbitration_id < arbitration_ids["ConbusUpperBound"]:
            self.log("Conbus", LogLevel.DEBUG)
            self.publish_conbus(msg)


    def publish_odom_feedback(self, msg):
        self.log(f"{msg.data.hex()}", LogLevel.DEBUG)
        delta_x, delta_y, delta_theta = struct.unpack('>hhh', msg.data)
        motor_feedback_msg = MotorFeedback()
        motor_feedback_msg.delta_x = delta_x / self.config.get("odom_feedback_scaler")
        motor_feedback_msg.delta_y = delta_y / self.config.get("odom_feedback_scaler")
        motor_feedback_msg.delta_theta = delta_theta / self.config.get("odom_feedback_scaler")

        self.motorFeedbackPublisher.publish(motor_feedback_msg)


    def publish_linear_pid_statistics(self, msg):
        forward_velocity, forward_velocity_setpoint, sideways_velocity, sideways_velocity_setpoint = struct.unpack(">hhhh", msg.data)
        linear_pid_statistics_msg = LinearPIDStatistics()
        linear_pid_statistics_msg.forward_velocity = forward_velocity
        linear_pid_statistics_msg.forward_velocity_setpoint - forward_velocity_setpoint
        linear_pid_statistics_msg.sideways_velocity = sideways_velocity
        linear_pid_statistics_msg.sideways_velocity_setpoint = sideways_velocity_setpoint

        self.linearPIDStatisticsPublisher.publish(linear_pid_statistics_msg)


    def publish_angular_pid_statistics(self, msg):
        angular_velocity, angular_velocity_setpoint = struct.unpack(">hh", msg.data)
        angular_pid_statistics_msg = AngularPIDStatistics()
        angular_pid_statistics_msg.angular_velocity = angular_velocity
        angular_pid_statistics_msg.angular_velocity_setpoint = angular_velocity_setpoint

        self.angularPIDStatisticsPublisher.publish(angular_pid_statistics_msg)


    def publish_motor_statistics_front_motors(self, msg):
        left_drive_motor_output, left_steer_motor_output, right_drive_motor_output, right_steer_motor_output= struct.unpack(">hhhh", msg.data)
        motor_statistics_front_motors = MotorStatisticsFrontMotors()
        motor_statistics_front_motors.left_drive_motor_output = left_drive_motor_output
        motor_statistics_front_motors.left_steer_motor_output = left_steer_motor_output
        motor_statistics_front_motors.right_drive_motor_output = right_drive_motor_output
        motor_statistics_front_motors.right_steer_motor_output = right_steer_motor_output

        self.motorStatisticsFrontMotorsPublisher.publish(motor_statistics_front_motors)


    def publish_motor_statistics_back_motors(self, msg):
        left_drive_motor_output, left_steer_motor_output, right_drive_motor_output, right_steer_motor_output= struct.unpack(">hhhh", msg.data)
        motor_statistics_back_motors = MotorStatisticsFrontMotors()
        motor_statistics_back_motors.left_drive_motor_output = left_drive_motor_output
        motor_statistics_back_motors.left_steer_motor_output = left_steer_motor_output
        motor_statistics_back_motors.right_drive_motor_output = right_drive_motor_output
        motor_statistics_back_motors.right_steer_motor_output = right_steer_motor_output

        self.motorStatisticsFrontMotorsPublisher.publish(motor_statistics_back_motors)


    def publish_conbus(self, msg):
        conbus = Conbus()
        conbus.id = msg.arbitration_id
        conbus.data = msg.data
        self.conbusPublisher.publish(conbus)


    # subscriber callbacks
    def on_safety_lights_received(self, msg:SafetyLights):
        safety_lights_packet = SafetyLightsPacket()
        safety_lights_packet.mode = msg.mode
        safety_lights_packet.brightness = msg.brightness
        safety_lights_packet.red = msg.red
        safety_lights_packet.green = msg.green
        safety_lights_packet.blue = msg.blue
        safety_lights_packet.blink_period = msg.blink_period

        data = bytes(safety_lights_packet)
        self.log(f"{data.hex()}", LogLevel.DEBUG)

        can_msg = can.Message(arbitration_id=arbitration_ids["SafetyLightsCommand"], data=data)
        
        try:
            self.can.send(can_msg)
        except can.CanError:
            pass


    def on_motor_input_received(self, msg:MotorInput):
        if self.get_device_state() != DeviceState.OPERATING:
            return
        data = struct.pack(">hhh", int(msg.forward_velocity * 1000), int(msg.sideways_velocity * 1000), int(msg.angular_velocity * 1000))
        can_msg = can.Message(arbitration_id = arbitration_ids["MotorsCommand"], data = data)
        self.log(f"{can_msg.data.hex()}", LogLevel.DEBUG)

        try:
            self.can.send(can_msg)
        except can.CanError:
            pass


    def on_conbus_received(self, msg:Conbus):
        self.log("Conbus received", LogLevel.DEBUG)
        if self.get_device_state() != DeviceState.OPERATING:
            return
        # try:
        data = bytes(msg.data)
        arbitration_id = msg.id

        can_msg = can.Message(arbitration_id = arbitration_id, data = data)
        
        try:
            self.log("Sending can message", LogLevel.DEBUG)
            self.can.send(can_msg)
        except can.CanError:
            pass
        # except Exception as e:
        #     self.log(f"{e}", LogLevel.DEBUG)
        #     pass
    
def main():
    rclpy.init()
    can_node = CanNode()
    rclpy.spin(can_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

