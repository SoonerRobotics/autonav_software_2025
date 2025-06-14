#!/usr/bin/env python3

import rclpy
from autonav_msgs.msg import ControllerInput, MotorInput, Position
import numpy as np
from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import AudibleFeedback, ControllerInput, ZeroEncoders
from enum import IntEnum

import time
import json
import os
import threading
import time


class ControllerMode(IntEnum):
    LOCAL = 0
    GLOBAL = 1
    

class Manual25Config:
    def __init__(self):
        self.max_forward_speed = -3
        self.max_sideways_speed = -3
        self.max_angular_speed = -np.pi
        self.odom_fudge_factor = 1
        self.sound_buffer = 0.5 # seconds
        self.main_song_path = '~/autonav_software_2025/music/lay.mp3'
        self.x_button_sound = '~/autonav_software_2025/music/vine-boom.mp3'
        self.right_button_sound = '~/autonav_software_2025/music/mine_xp.mp3'

class Manual25Node(Node):
    def __init__(self):
        super().__init__('autonav_manual')
        self.config = Manual25Config()
        # self.log("Manual 25 node __init__", LogLevel.INFO)


    def init(self):
        self.log("Manual 25 node init", LogLevel.INFO)
        self.mode = ControllerMode.LOCAL
        self.orientation = 0
        self.last_time = 0
        self.delta_t = 0
        self.new_time = time.time()
        
        # self.max_forward_speed = 1
        # self.max_angular_speed = np.pi/4

        self.controller_state = {}

        

        self.audio_manager = threading.Thread(target=self.manage_audio)
        self.audio_manager.daemon = True
        self.audio_manager.start()

        self.set_device_state(DeviceState.WARMING)

        self.controllerSubscriber = self.create_subscription(
            ControllerInput,
            '/autonav/controller_input',
            self.input_callback,
            10
        )
        
        self.positionSubscriber = self.create_subscription(
                Position,
                '/autonav/position',
                self.position_callback,
                10
        )

        self.zeroEncodersPublisher = self.create_publisher(
            ZeroEncoders,
            'autonav/zero_encoders',
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

    def apply_config(self, config):
        self.config.max_forward_speed = config["max_forward_speed"]
        self.config.max_sideways_speed = config["max_sideways_speed"]
        self.config.max_angular_speed = config["max_angular_speed"]
        self.config.odom_fudge_factor = config["odom_fudge_factor"]
        self.config.sound_buffer = config["sound_buffer"]
        self.config.main_song_path = config["main_song_path"]
        self.config.x_button_sound = config["x_button_sound"]

    def input_callback(self, msg):
        self.new_time = time.time()
        self.delta_t = self.new_time - self.last_time

        if self.get_device_state() != DeviceState.OPERATING:
            self.set_device_state(DeviceState.OPERATING)
        self.deserialize_controller_state(msg)

        self.change_controller_mode()
        self.change_system_state()
        self.handle_encoders()

        # # self.log(f"orientation: {self.orientation}")
        # # local vs. global toggle

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

        elif self.controller_state["btn_south"] == 1.0:
            self.orientation = 0
            self.mode = ControllerMode.LOCAL

    def on_system_state_updated(self, old, new):
        if new != SystemState.MANUAL and old == SystemState.MANUAL:
            motor_msg = MotorInput()
            self.motorPublisher.publish(motor_msg)

        if new == SystemState.DISABLED and old != SystemState.DISABLED:
            self.push_safety_lights(255, 255, 255, 3, 0)

        if new == SystemState.MANUAL and old != SystemState.MANUAL:
            self.push_safety_lights(255, 255, 0, 0, 0)

    def change_system_state(self):
        if self.controller_state['btn_east'] == 1.0:
            self.set_system_state(SystemState.SHUTDOWN)
        elif self.controller_state['btn_start'] == 1.0:
            self.set_system_state(SystemState.MANUAL)
        elif self.controller_state['btn_mode'] == 1.0:
            self.set_system_state(SystemState.AUTONOMOUS)
        elif self.controller_state['btn_select'] == 1.0:
            self.set_system_state(SystemState.DISABLED)

    
    def handle_encoders(self):
        if self.controller_state['btn_tl'] == 1.0:
            # self.log("zeroing the encoders", LogLevel.INFO)
            number_of_absolute_encoders = 4
            for i in range(number_of_absolute_encoders):
                encoder_msg = ZeroEncoders()
                encoder_msg.which_encoder = i
                self.zeroEncodersPublisher.publish(encoder_msg)


    def compose_motorinput_message_local(self):
        if self.system_state != SystemState.MANUAL:
            return
        
        forward_velocity = self.normalize(self.controller_state["abs_y"], -self.config.max_forward_speed, self.config.max_forward_speed, -1.0, 1.0)
        sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.config.max_sideways_speed, self.config.max_sideways_speed, -1.0, 1.0)
        angular_velocity = self.normalize(self.controller_state["abs_z"], -self.config.max_angular_speed, self.config.max_angular_speed, -1.0, 1.0)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity
        motor_msg.sideways_velocity = sideways_velocity
        motor_msg.angular_velocity = angular_velocity
        self.motorPublisher.publish(motor_msg)
    

    # https://math.stackexchange.com/questions/2895880/inversion-of-rotation-matrix
    def compose_motorinput_message_global(self):
        if self.system_state != SystemState.MANUAL:
            return
        
        forward_velocity = self.normalize(self.controller_state["abs_y"], -self.config.max_forward_speed, self.config.max_forward_speed, 1.0, -1.0)
        sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.config.max_sideways_speed, self.config.max_sideways_speed, -1.0, 1.0)
        angular_velocity = self.normalize(self.controller_state["abs_z"], -self.config.max_angular_speed, self.config.max_angular_speed, 1.0, -1.0)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity * np.cos(self.orientation) + -1 * sideways_velocity * np.sin(self.orientation)
        motor_msg.sideways_velocity = sideways_velocity * np.cos(self.orientation) + forward_velocity * np.sin(self.orientation)
        motor_msg.angular_velocity = angular_velocity
        self.motorPublisher.publish(motor_msg)


    def play_sound(self):
        self.new_time = time.time()
        self.delta_t = self.new_time - self.last_time

        if self.get_device_state() != DeviceState.READY and self.get_device_state() != DeviceState.OPERATING:
            return

        if self.controller_state == {}:
            return

        if self.delta_t < self.config.sound_buffer:
            return

        if self.controller_state['btn_west'] == 1:
            audible_feedback = AudibleFeedback()
            audible_feedback.filename = os.path.expanduser(self.config.x_button_sound)
            self.audibleFeedbackPublisher.publish(audible_feedback)
            self.last_time = time.time()

        elif self.controller_state['abs_hat0y'] == -1:
            audible_feedback = AudibleFeedback()
            audible_feedback.filename= os.path.expanduser(self.config.main_song_path)
            audible_feedback.main_track = True
            self.audibleFeedbackPublisher.publish(audible_feedback)
            self.last_time = time.time()

        elif self.controller_state['btn_south'] == 1:
            audible_feedback = AudibleFeedback()
            audible_feedback.filename = os.path.expanduser('~/Documents/robot_relative.mp3')
            self.audibleFeedbackPublisher.publish(audible_feedback)
            self.last_time = time.time()

        elif self.controller_state['btn_north'] == 1:
            audible_feedback = AudibleFeedback()
            audible_feedback.filename = os.path.expanduser('~/Documents/field_oriented.mp3')
            self.audibleFeedbackPublisher.publish(audible_feedback)
            self.last_time = time.time()
        
        elif self.controller_state['btn_tr'] == 1:
            audible_feedback = AudibleFeedback()
            audible_feedback.filename = os.path.expanduser(self.config.right_button_sound)
            self.audibleFeedbackPublisher.publish(audible_feedback)
            self.last_time = time.time()

    def manage_audio(self):
        self.log("Audio manager started", LogLevel.INFO)
        while rclpy.ok():
            if self.get_device_state() != DeviceState.READY and self.get_device_state() != DeviceState.OPERATING:
                time.sleep(0.5)
                continue

            if self.controller_state == {}:
                time.sleep(0.5)
                continue

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

            self.play_sound()
            time.sleep(0.5)

        
    def position_callback(self, msg: Position):
        self.orientation = msg.theta
        
        return

def main(args=None):
    rclpy.init()
    node = Manual25Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
