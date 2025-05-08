#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, SwerveFeedback, ZeroEncoders, MotorFeedback, SwerveAbsoluteFeedback
from autonav_shared.types import LogLevel, DeviceState, SystemState

import can
from swerve.swerve_drive import SUSwerveDrive, SUSwerveDriveState
from swerve.swerve_module import SUSwerveDriveModule
from swerve.can_spark_max import CanSparkMax
from swerve.swerve_config import *

#FIXME CanConfig isn't working right right now
class CanNodeConfig:
    def __init__(self):
        self.canable_filepath = "/dev/ttyACM0" #TODO this may or may not be right, there are two CANables so not sure

class SparkMAXNode(Node):
    def __init__(self):
        super().__init__("sparkmax_can_node")
        self.write_config(CanNodeConfig())
        self.can = None
        self.motors = []
        self.hasConfigured = False
    
    def init(self):
        self.set_device_state(DeviceState.WARMING)

        # make the CAN object
        self.can = can.ThreadSafeBus(bustype="slcan", channel="/dev/ttyACM0", bitrate=1_000_000) # FRC CAN runs at 1 Mbit/sec

        # ROS motor message callback
        self.motorInputSubscriber = self.create_subscription(MotorInput, "/autonav/motor_input", self.on_motor_input_received, 20)
        self.motorFeedbackPublisher = self.create_publisher(MotorFeedback, "/autonav/motor_feedback", 20)

        # feedback publisher
        self.absoluteEncoderPublisher = self.create_publisher(SwerveAbsoluteFeedback, "/autonav/swerve/absolute", 20)
        self.swerveFeedbackPublisher = self.create_publisher(SwerveFeedback, "/autonav/swerve/feedback", 20)

        # Periodic heartbeat to keep motors enabled
        self.heartbeat_timer = self.create_timer(0.05, self.send_heartbeat)
        self.feedback_timer = self.create_timer(0.1, self.send_motor_feedbacK)

        self.motors = [
            CanSparkMax(1, self.can), # drive
            CanSparkMax(2, self.can), # angle
            CanSparkMax(3, self.can), # angle
            CanSparkMax(4, self.can), # drive
            CanSparkMax(5, self.can), # drive
            CanSparkMax(6, self.can), # angle
            CanSparkMax(7, self.can), # angle
            CanSparkMax(8, self.can), # drive
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

    def send_motor_feedbacK(self):
        feedback = SwerveAbsoluteFeedback()
        feedback.position_fl = self.motors[6].getAbsolutePosition() #7
        feedback.position_fr = self.motors[5].getAbsolutePosition() #6
        feedback.position_bl = self.motors[2].getAbsolutePosition() #3
        feedback.position_br = self.motors[1].getAbsolutePosition() #2

        # Publish the feedbacks
        self.absoluteEncoderPublisher.publish(feedback)

    def on_motor_input_received(self, msg: MotorInput):
        if self.get_device_state() != DeviceState.OPERATING:
            self.set_device_state(DeviceState.OPERATING)

        swerve_feedback = self.swerve.updateState(SUSwerveDriveState(
            msg.sideways_velocity,
            -msg.forward_velocity,
            msg.angular_velocity
        ), 0.02, on_motor_setpoint_callback=self.on_motor_setpoint_callback)

        # publish feedback
        feedback_msg = MotorFeedback()
        feedback_msg.delta_x = swerve_feedback.x_vel / 10
        feedback_msg.delta_y = swerve_feedback.y_vel / 10
        feedback_msg.delta_theta = swerve_feedback.angular_vel / 10
        self.motorFeedbackPublisher.publish(feedback_msg)

    def on_motor_setpoint_callback(self, module, desired_x_vel, desired_y_vel, desired_angular_vel, measured_x_vel, measured_y_vel, measured_angular_vel):
        feedback_msg = SwerveFeedback()
        feedback_msg.module = module
        feedback_msg.desired_x_vel = desired_x_vel
        feedback_msg.desired_y_vel = desired_y_vel
        feedback_msg.desired_angular_vel = desired_angular_vel
        feedback_msg.measured_x_vel = measured_x_vel
        feedback_msg.measured_y_vel = measured_y_vel
        feedback_msg.measured_angular_vel = measured_angular_vel
        self.swerveFeedbackPublisher.publish(feedback_msg)

    def reconnect_can(self):
        try:
            self.log("Attempting to reconnect SparkMAX CAN bus...", LogLevel.INFO)
            self.can = can.ThreadSafeBus(bustype="slcan", channel=self.config.canable_filepath, bitrate=1_000_000) # FRC CAN runs at 1 Mbit/sec
            self.reconnect_timer.destroy() # can is connected, don't need to keep trying
            self.set_device_state(DeviceState.READY)
        except:
            self.set_device_state(DeviceState.ERROR)

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
