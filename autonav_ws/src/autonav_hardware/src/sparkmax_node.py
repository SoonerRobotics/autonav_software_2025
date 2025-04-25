#!/usr/bin/env python3

import rclpy
from autonav_shared.node import Node
from autonav_msgs.msg import MotorInput, MotorStatistics, ZeroEncoders, MotorFeedback, SwerveAbsoluteFeedback
from autonav_shared.types import LogLevel, DeviceState, SystemState

import can
from swerve.swerve_drive import SUSwerveDrive, SUSwerveDriveState
from swerve.swerve_module import SUSwerveDriveModule
from swerve.can_spark_max import CanSparkMax
from swerve.swerve_config import *

#FIXME CanConfig isn't working right right now
class CanNodeConfig:
    def __init__(self):
        self.canable_filepath = "/dev/ttyACM1" #TODO this may or may not be right, there are two CANables so not sure

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

        # Periodic heartbeat to keep motors enabled
        self.heartbeat_timer = self.create_timer(0.05, self.send_heartbeat)
        self.feedback_timer = self.create_timer(0.2, self.send_motor_feedbacK)

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
        except Exception as e:
            self.set_device_state(DeviceState.ERROR)
            self.log(f"Can't connect to SparkMAX CAN: {e}", LogLevel.ERROR)
            self.reconnect_timer = self.create_timer(5, self.reconnect_can)

        self.set_device_state(DeviceState.READY)
    
    #TODO: is there a better way to do this? maybe have a timer in each object itself? 
    # I feel like this should be more abstracted away, at least from this file
    def send_heartbeat(self):
        for idx, motor in enumerate(self.motors):
            motor.sendHeartbeat()

    def send_motor_feedbacK(self):
        feedback_fl = SwerveAbsoluteFeedback()
        feedback_fl.module = 0
        feedback_fl.position = self.motors[0].getAbsolutePosition()

        feedback_fr = SwerveAbsoluteFeedback()
        feedback_fr.module = 1
        feedback_fr.position = self.motors[3].getAbsolutePosition()

        feedback_bl = SwerveAbsoluteFeedback()
        feedback_bl.module = 2
        feedback_bl.position = self.motors[4].getAbsolutePosition()

        feedback_br = SwerveAbsoluteFeedback()
        feedback_br.module = 3
        feedback_br.position = self.motors[7].getAbsolutePosition()

        # Publish the feedbacks
        self.absoluteEncoderPublisher.publish(feedback_fl)
        self.absoluteEncoderPublisher.publish(feedback_fr)
        self.absoluteEncoderPublisher.publish(feedback_bl)
        self.absoluteEncoderPublisher.publish(feedback_br)

    def on_motor_input_received(self, msg: MotorInput):
        if self.get_device_state() != DeviceState.OPERATING:
            self.set_device_state(DeviceState.OPERATING)

        self.log(f"we do a swerve: {msg.forward_velocity}")

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

        # publish feedback
        feedback_msg = MotorFeedback()
        feedback_msg.delta_x = swerve_feedback.x_vel
        feedback_msg.delta_y = swerve_feedback.y_vel
        feedback_msg.delta_theta = swerve_feedback.angular_vel
        self.motorFeedbackPublisher.publish(feedback_msg)

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
