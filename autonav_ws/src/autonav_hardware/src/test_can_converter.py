#!/usr/bin/env python3

import can
import threading
from swerve.swerve_drive import SUSwerveDrive, SUSwerveDriveState
from swerve.swerve_module import SUSwerveDriveModule
from swerve.can_spark_max import CanSparkMax
from swerve.swerve_config import *


CAN_PATH = "/dev/ttyACM0"
CAN_SPEED = 1_000_000


class SparkMAXNode():
    def __init__(self):
        self.can = None
        self.motors = []
    
    def init(self):
        # make the CAN object
        self.can = can.ThreadSafeBus(bustype="slcan", channel=CAN_PATH, bitrate=CAN_SPEED)
        self.canReadThread = threading.Thread(target=self.can_reader)
        self.canReadThread.daemon = True
        self.canReadThread.start()

        self.motors = [
            CanSparkMax(1, self.can, self), # drive
            CanSparkMax(2, self.can, self), # angle
            CanSparkMax(3, self.can, self), # angle
            CanSparkMax(4, self.can, self), # drive
            CanSparkMax(5, self.can, self), # drive
            CanSparkMax(6, self.can, self), # angle
            CanSparkMax(7, self.can, self), # angle
            CanSparkMax(8, self.can, self), # drive
        ]

        self.modules = (
            SUSwerveDriveModule(front_left_module_config, self.motors[0], self.motors[1]),
            SUSwerveDriveModule(front_right_module_config, self.motors[3], self.motors[2]),
            SUSwerveDriveModule(back_left_module_config, self.motors[4], self.motors[5]),
            SUSwerveDriveModule(back_right_module_config, self.motors[7], self.motors[6]),
            swerve_config
        )

        # to the uninitiated: this is not a pointer. this is python argument unpacking
        self.swerve_drive = SUSwerveDrive(*self.modules)
    
    #TODO: is there a better way to do this? maybe have a timer in each object itself? 
    # I feel like this should be more abstracted away, at least from this file
    def send_heartbeat(self):
        for idx, motor in enumerate(self.motors):
            motor.sendHeartbeat()

    def can_reader(self):
        while True:
            try:
                msg = self.can.recv()
                if msg is not None:
                    for motor in self.motors:
                        motor.canCallback(msg)
            except can.CanError as e:
                print(f"CAN error: {e}")

    def send_motor_feedbacK(self):
        feedback = SwerveAbsoluteFeedback()
        feedback.position_fl = self.motors[1].getAbsolutePosition() #7
        feedback.position_fr = self.motors[2].getAbsolutePosition() #6
        feedback.position_bl = self.motors[5].getAbsolutePosition() #3
        feedback.position_br = self.motors[6].getAbsolutePosition() #2

        self.absoluteEncoderPublisher.publish(feedback)

    def on_motor_input_received(self, msg: MotorInput):
        if self.get_device_state() != DeviceState.OPERATING:
            self.set_device_state(DeviceState.OPERATING)

        # tony gives us forwrd and angular in the wrong direction :(
        swerve_feedback = self.swerve_drive.updateState(SUSwerveDriveState(
            msg.forward_velocity,
            msg.sideways_velocity,
            msg.angular_velocity
        ), 0.01)

        # publish feedback
        feedback_msg = MotorFeedback()
        feedback_msg.delta_x = swerve_feedback.delta_x
        feedback_msg.delta_y = swerve_feedback.delta_y
        feedback_msg.delta_theta = swerve_feedback.delta_theta
        self.motorFeedbackPublisher.publish(feedback_msg)
    
    def reconnect_can(self):
        try:
            self.can = can.ThreadSafeBus(bustype="slcan", channel=CAN_PATH, bitrate=CAN_SPEED)
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
