#!/usr/bin/env python3

import rclpy
from autonav_msgs.msg import ControllerInput, MotorInput, MotorFeedback
import numpy as np
from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import AudibleFeedback, ControllerInput
from enum import IntEnum
import time
import json
import os

class ControllerMode(IntEnum):
    LOCAL = 0
    GLOBAL = 1


class Manual25Config:
    def __init__(self):
        self.max_forward_speed = 3
        self.max_sideways_speed = 3
        self.max_angular_speed = np.pi
        self.odom_fudge_factor = 1
        self.sound_buffer = 0.5 # seconds


class Manual25Node(Node):
    def __init__(self):
        super().__init__('manual25_node')
        self.write_config(Manual25Config())
        self.log("bigger pepe")


    def init(self):
        self.mode = ControllerMode.LOCAL
        self.orientation = 0
        self.last_time = 0
        self.delta_t = 0
        self.new_time = time.time()
        # self.max_forward_speed = 1
        # self.max_angular_speed = np.pi/4

        self.controller_state = {}

        self.set_device_state(DeviceState.WARMING)

        self.controllerSubscriber = self.create_subscription(
            ControllerInput,
            '/autonav/controller_input',
            self.input_callback,
            10
        )

        self.motorSubscription = self.create_subscription(
            MotorFeedback,
            '/autonav/motor_feedback',
            self.on_motor_feedback,
            10
        )
        
        self.motorPublisher = self.create_publisher(
            MotorInput,
            '/autonav/motor_input',
            10
        )

        self.audibleFeedbackPublisher = self.create_publisher(
            AudibleFeedback,
            '/autonav/audible_feedback',
            10
        )

        self.controllerSubscriber  # prevent unused variable warning


    def input_callback(self, msg):
        self.new_time = time.time()
        self.delta_t = self.new_time - self.last_time

        self.set_device_state(DeviceState.OPERATING)
        self.deserialize_controller_state(msg)
        # self.log(f"I heard: {str(self.controller_state)}")

        self.change_controller_mode()
        self.change_system_state()

        if self.delta_t > self.config.get("sound_buffer"):
            self.play_sound()
            self.last_time = time.time()
        
        self.manage_audio()


        self.log(f"orientation: {self.orientation}")
        # local vs. global toggle

        if self.mode == ControllerMode.LOCAL:
            self.compose_motorinput_message_local()
        elif self.mode == ControllerMode.GLOBAL:
            self.compose_motorinput_message_global()

        
    def deserialize_controller_state(self, msg):
        attributes = [n for n in dir(msg) if not (n.startswith('__') or n.startswith('_'))]
        attributes.remove('SLOT_TYPES')
        attributes.remove('get_fields_and_field_types')
        for attribute in attributes:
            self.controller_state[attribute] = getattr(msg, attribute)


    def normalize(self, input, output_start, output_end, input_start, input_end):
        return output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
    
    
    def change_controller_mode(self):
        if self.controller_state["btn_north"] == 1.0:
            self.orientation = 0
            self.mode = ControllerMode.GLOBAL
            self.log(f'changed controller mode to {self.mode}')
        elif self.controller_state["btn_south"] == 1.0:
            self.orientation = 0
            self.mode = ControllerMode.LOCAL
            self.log(f'changed controller mode to {self.mode}')
            

    def change_system_state(self):
        new_system_state = self.system_state
        if self.controller_state['btn_east'] == 1.0:
            new_system_state = SystemState.SHUTDOWN
            self.log(f'Setting system state to {new_system_state}')
            self.set_system_state(new_system_state)
            
        elif self.controller_state['btn_start'] == 1.0:
            new_system_state = SystemState.MANUAL
            self.log(f'Setting system state to {new_system_state}')
            self.set_system_state(new_system_state)

        elif self.controller_state['btn_mode'] == 1.0:
            new_system_state = SystemState.AUTONOMOUS
            self.log(f'Setting system state to {new_system_state}')
            self.set_system_state(new_system_state)

        elif self.controller_state['btn_select'] == 1.0:
            new_system_state = SystemState.DISABLED
            self.log(f'Setting system state to {new_system_state}')
            self.set_system_state(new_system_state)


    def compose_motorinput_message_local(self):
        forward_velocity = 0.0
        sideways_velocity = 0.0
        angular_velocity = 0.0
        if self.system_state == SystemState.MANUAL:
            forward_velocity = self.normalize(self.controller_state["abs_y"], -self.config.get("max_forward_speed"), self.config.get("max_forward_speed"), 1.0, -1.0)
            sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.config.get("max_sideways_speed"), self.config.get("max_sideways_speed"), -1.0, 1.0)
            angular_velocity = self.normalize(self.controller_state["abs_z"], -self.config.get("max_angular_speed"), self.config.get("max_angular_speed"), 1.0, -1.0)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity
        motor_msg.sideways_velocity = sideways_velocity
        motor_msg.angular_velocity = angular_velocity

        self.motorPublisher.publish(motor_msg)
    

    # https://math.stackexchange.com/questions/2895880/inversion-of-rotation-matrix
    def compose_motorinput_message_global(self):
        forward_velocity = 0.0
        sideways_velocity = 0.0
        angular_velocity = 0.0
        if self.system_state == SystemState.MANUAL:
            forward_velocity = self.normalize(self.controller_state["abs_y"], -self.config.get("max_forward_speed"), self.config.get("max_forward_speed"), 1.0, -1.0)
            sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.config.get("max_sideways_speed"), self.config.get("max_sideways_speed"), -1.0, 1.0)
            angular_velocity = self.normalize(self.controller_state["abs_z"], -self.config.get("max_angular_speed"), self.config.get("max_angular_speed"), 1.0, -1.0)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity * np.cos(self.orientation) + sideways_velocity * np.sin(self.orientation)
        motor_msg.sideways_velocity = -1 * sideways_velocity * np.cos(self.orientation) + forward_velocity * np.sin(self.orientation)
        motor_msg.angular_velocity = angular_velocity

        self.motorPublisher.publish(motor_msg)


    def on_motor_feedback(self, msg:MotorFeedback):
        delta_theta = msg.delta_theta * self.config.get("odom_fudge_factor")
        self.orientation += delta_theta


    def play_sound(self):
        if self.controller_state['btn_west'] == 1:
            audible_feedback = AudibleFeedback()
            audible_feedback.filename = os.path.expanduser('~/Documents/vine-boom.mp3')
            self.audibleFeedbackPublisher.publish(audible_feedback)

        if self.controller_state['abs_hat0y'] == -1:
            audible_feedback = AudibleFeedback()
            audible_feedback.filename= os.path.expanduser('~/Documents/vivalavida.wav')
            audible_feedback.main_track = True
            self.audibleFeedbackPublisher.publish(audible_feedback)


    def manage_audio(self):
        if self.controller_state['abs_hat0y'] == 1:
            audible_feedback = AudibleFeedback()
            audible_feedback.stop_all = True
            self.audibleFeedbackPublisher.publish(audible_feedback)

        if self.controller_state['abs_hat0x'] == -1:
            audible_feedback = AudibleFeedback()
            audible_feedback.pause_all = True
            self.audibleFeedbackPublisher.publish(audible_feedback)

        if self.controller_state['abs_hat0x'] == 1:
            audible_feedback = AudibleFeedback()
            audible_feedback.unpause_all = True
            self.audibleFeedbackPublisher.publish(audible_feedback)


def main(args=None):
    rclpy.init()
    node = Manual25Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()