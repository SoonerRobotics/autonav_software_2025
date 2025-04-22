#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorStatistics, ZeroEncoders, MotorFeedback
from autonav_shared.types import LogLevel, DeviceState, SystemState

import can
from swerve.swerve_drive import SUSwerveDrive, SUSwerveDriveState
from swerve.swerve_module import SUSwerveDriveModule
from swerve.can_spark_max import CanSparkMax
from swerve.swerve_config import *

#FIXME CanConfig doesn't do anything right now
class CanConfig:
    def __init__(self):
        self.canable_filepath = "/dev/ttyACM0"

class SparkMAXNode(Node):
    def __init__(self):
        super().__init__("sparkmax_can_node")
        self.write_config(CanConfig())
        self.can = None
        self.motors = []
        self.hasConfigured = False
    
    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # make the CAN object
        self.can = can.ThreadSafeBus(bustype="slcan", channel=self.config.get("canable_filepath"), bitrate=1_000_000) # FRC CAN runs at 1 Mbit/sec

        # ROS motor message callback
        self.motorInputSubscriber = self.create_subscription(MotorInput, "/autonav/motor_input", self.on_motor_input_received, 20)
        self.motorFeedbackPublisher = self.create_publisher(MotorFeedback, "/autonav/motor_feedback", 20)

        # Periodic heartbeat to keep motors enabled
        self.heartbeat_timer = self.create_timer(0.05, self.send_heartbeat)

        self.motors = [
            CanSparkMax(1, self.can),
            CanSparkMax(2, self.can),
            CanSparkMax(3, self.can),
            CanSparkMax(4, self.can),
            CanSparkMax(5, self.can),
            CanSparkMax(6, self.can),
            CanSparkMax(7, self.can),
            CanSparkMax(8, self.can),
        ]

        self.modules = (
            SUSwerveDriveModule(front_left_module_config, self.motors[0], self.motors[1]),
            SUSwerveDriveModule(front_right_module_config, self.motors[3], self.motors[2]),
            SUSwerveDriveModule(back_left_module_config, self.motors[4], self.motors[5]),
            SUSwerveDriveModule(back_right_module_config, self.motors[7], self.motors[6]),
            swerve_config
        )

        # to the uninitiated: this is not a pointer. this is python argument unpacking
        self.swerve = SUSwerveDrive(*self.modules)
        
        self.set_device_state(DeviceState.READY)
    
    #TODO: is there a better way to do this? maybe have a timer in each object itself? 
    # I feel like this should be more abstracted away, at least from this file
    def send_heartbeat(self):
        for idx, motor in enumerate(self.motors):
            motor.sendHeartbeat()

    def on_motor_input_received(self, msg: MotorInput):
        if self.get_device_state() != DeviceState.OPERATING:
            self.set_device_state(DeviceState.OPERATING)

        swerve_feedback = self.swerve.updateState(SUSwerveDriveState(
            msg.sideways_velocity,
            -msg.forward_velocity,
            msg.angular_velocity
        ), 0.02)

        # publish feedback
        feedback_msg = MotorFeedback()
        feedback_msg.delta_x = swerve_feedback.x_vel
        feedback_msg.delta_y = swerve_feedback.y_vel
        feedback_msg.delta_theta = swerve_feedback.angular_vel
        self.motorFeedbackPublisher.publish(feedback_msg)

def main():
    rclpy.init()
    can_node = SparkMAXNode()
    try:
        rclpy.spin(can_node)
    except KeyboardInterrupt:
        # shutdown the CAN
        for motor in can_node.motors:
            motor.notifier.stop()
            motor.can.shutdown()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
