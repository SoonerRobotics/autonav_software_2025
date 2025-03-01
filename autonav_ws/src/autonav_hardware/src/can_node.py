#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorFeedback, SafetyLights, Ultrasonic, Conbus, CanStats
from autonav_msgs.msg import LinearPIDStatistics, AngularPIDStatistics, MotorStatistics
from autonav_shared.types import LogLevel, DeviceState, SystemState
import can
import threading
import struct
from ctypes import Structure, c_uint8


class CanConfig:
    def __init__(self):
        self.canable_filepath = "/dev/ttyACM0"
        self.odom_feedback_scaler = 10000
        self.linear_pid_scaler = 1000
        self.angular_pid_scaler = 1000
        self.front_motor_scaler = 1000
        self.back_motor_scaler = 1000


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


class CanNode(Node):
    def __init__(self):
        super().__init__("CAN_node")
        self.write_config(CanConfig())
        self.can_stats_record = CanStats()

        # can
        self.can = None

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
            MotorStatistics,
            "/autonav/motor_statistics_front_motors",
            20
        )
        self.motorStatisticsBackMotorsPublisher = self.create_publisher(
            MotorStatistics,
            "/autonav/motor_statistics_back_motors",
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
        while rclpy.ok():
            if self.get_device_state() != DeviceState.READY and self.get_device_state() != DeviceState.OPERATING:
                continue
            if self.can is not None:
                try:
                    msg = self.can.recv(timeout=0.01)
                    if msg is not None:
                        self.onCanMessageReceived(msg)
                except Exception as e:
                    self.log(f"Received erroneous CAN message from hardware {e}", LogLevel.ERROR)
    

    def onCanMessageReceived(self, msg):
        self.can_stats_record.rx = self.can_stats_record.rx + 1
        arbitration_id = msg.arbitration_id

        if arbitration_id == arbitration_ids["EStop"]:
            self.set_system_state(SystemState.DISABLED)

        elif arbitration_id == arbitration_ids["MobilityStart"]:
            self.set_mobility(True)
        
        elif arbitration_id == arbitration_ids["MobilityStop"]:
            self.set_mobility(False)

        elif arbitration_id == arbitration_ids["OdometryFeedback"]:
            self.publish_odom_feedback(msg)

        elif arbitration_id == arbitration_ids["ObjectDetection"]:
            self.publish_object_detection(msg)

        elif arbitration_id == arbitration_ids["LinearPIDStatistics"]:
            self.publish_linear_pid_statistics(msg)
    
        elif arbitration_id == arbitration_ids["AngularPIDStatistics"]:
            self.publish_angular_pid_statistics(msg)

        elif arbitration_id == arbitration_ids["MotorStatisticsFrontMotors"]:
            self.publish_motor_statistics_front_motors(msg)
        
        elif arbitration_id == arbitration_ids["MotorStatisticsBackMotors"]:
            self.publish_motor_statistics_back_motors(msg)

        elif arbitration_id >= arbitration_ids["ConbusLowerBound"] and arbitration_id < arbitration_ids["ConbusUpperBound"]:
            self.publish_conbus(msg)

        else:
            self.log(f"Received CAN message with invalid id: {arbitration_id}, data: {msg.data}", LogLevel.ERROR)


    def publish_odom_feedback(self, msg):
        delta_x, delta_y, delta_theta = struct.unpack('>hhh', msg.data)
        motor_feedback_msg = MotorFeedback()
        motor_feedback_msg.delta_x = delta_x / self.config.get("odom_feedback_scaler")
        motor_feedback_msg.delta_y = delta_y / self.config.get("odom_feedback_scaler")
        motor_feedback_msg.delta_theta = delta_theta / self.config.get("odom_feedback_scaler")

        self.motorFeedbackPublisher.publish(motor_feedback_msg)


    def publish_object_detection(self, msg):
        ultrasonic_msg = Ultrasonic()
        data = bytes(msg.data)
        ultrasonic_msg.id = data[0]
        ultrasonic_msg.distance = (data[1] << 8) | data[2]

        self.objectDetectionPublisher.publish(ultrasonic_msg)


    def publish_linear_pid_statistics(self, msg):
        forward_velocity, forward_velocity_setpoint, sideways_velocity, sideways_velocity_setpoint = struct.unpack(">hhhh", msg.data)
        linear_pid_statistics_msg = LinearPIDStatistics()
        linear_pid_statistics_msg.forward_velocity = int(forward_velocity / self.config.get("linear_pid_scaler"))
        linear_pid_statistics_msg.forward_velocity_setpoint = int(forward_velocity_setpoint / self.config.get("linear_pid_scaler"))
        linear_pid_statistics_msg.sideways_velocity = int(sideways_velocity / self.config.get("linear_pid_scaler"))
        linear_pid_statistics_msg.sideways_velocity_setpoint = int(sideways_velocity_setpoint / self.config.get("linear_pid_scaler"))

        self.linearPIDStatisticsPublisher.publish(linear_pid_statistics_msg)


    def publish_angular_pid_statistics(self, msg):
        angular_velocity, angular_velocity_setpoint = struct.unpack(">hh", msg.data)
        angular_pid_statistics_msg = AngularPIDStatistics()
        angular_pid_statistics_msg.angular_velocity = int(angular_velocity / self.config.get("angular_pid_scaler"))
        angular_pid_statistics_msg.angular_velocity_setpoint = int(angular_velocity_setpoint / self.config.get("angular_pid_scaler"))

        self.angularPIDStatisticsPublisher.publish(angular_pid_statistics_msg)


    def publish_motor_statistics_front_motors(self, msg):
        left_drive_motor_output, left_steer_motor_output, right_drive_motor_output, right_steer_motor_output= struct.unpack(">hhhh", msg.data)
        motor_statistics = MotorStatistics()
        motor_statistics.left_drive_motor_output = int(left_drive_motor_output / self.config.get("front_motor_scaler"))
        motor_statistics.left_steer_motor_output = int(left_steer_motor_output / self.config.get("front_motor_scaler"))
        motor_statistics.right_drive_motor_output = int(right_drive_motor_output / self.config.get("front_motor_scaler"))
        motor_statistics.right_steer_motor_output = int(right_steer_motor_output / self.config.get("front_motor_scaler"))

        self.motorStatisticsFrontMotorsPublisher.publish(motor_statistics)


    def publish_motor_statistics_back_motors(self, msg):
        left_drive_motor_output, left_steer_motor_output, right_drive_motor_output, right_steer_motor_output= struct.unpack(">hhhh", msg.data)
        motor_statistics = MotorStatistics()
        motor_statistics.left_drive_motor_output = int(left_drive_motor_output / self.config.get("back_motor_scaler"))
        motor_statistics.left_steer_motor_output = int(left_steer_motor_output / self.config.get("back_motor_scaler"))
        motor_statistics.right_drive_motor_output = int(right_drive_motor_output / self.config.get("back_motor_scaler"))
        motor_statistics.right_steer_motor_output = int(right_steer_motor_output / self.config.get("back_motor_scaler"))

        self.motorStatisticsBackMotorsPublisher.publish(motor_statistics)


    def publish_conbus(self, msg):
        conbus = Conbus()
        conbus.id = msg.arbitration_id
        conbus.data = msg.data
        self.conbusPublisher.publish(conbus)


    # subscriber callbacks
    def on_safety_lights_received(self, msg:SafetyLights):
        self.can_stats_record.tx = self.can_stats_record.tx + 1

        safety_lights_packet = SafetyLightsPacket()
        safety_lights_packet.mode = msg.mode
        safety_lights_packet.brightness = msg.brightness
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
        self.can_stats_record.tx = self.can_stats_record.tx + 1

        if self.get_device_state() != DeviceState.OPERATING:
            return
        data = struct.pack(">hhh", int(msg.forward_velocity * 1000), int(msg.sideways_velocity * 1000), int(msg.angular_velocity * 1000))
        can_msg = can.Message(arbitration_id = arbitration_ids["MotorsCommand"], data = data)

        try:
            self.can.send(can_msg)
        except can.CanError:
            pass


    def on_conbus_received(self, msg:Conbus):
        self.can_stats_record.tx = self.can_stats_record.tx + 1

        if self.get_device_state() != DeviceState.OPERATING:
            return
        # try:
        data = bytes(msg.data)
        arbitration_id = msg.id

        can_msg = can.Message(arbitration_id = arbitration_id, data = data)
        
        try:
            self.can.send(can_msg)
        except can.CanError:
            pass
        # except Exception as e:
        #     self.log(f"{e}", LogLevel.DEBUG)
        #     pass
    

    def publish_can_stats(self):
        self.can_stats_publisher.publish(self.can_stats_record)


def main():
    rclpy.init()
    can_node = CanNode()
    rclpy.spin(can_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

