#!/usr/bin/env python3

import rclpy
import json
from autonav_msgs.msg import ControllerInput, MotorInput
from types import SimpleNamespace
import numpy as np
from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import ControllerInput
from enum import IntEnum
import time

class ControllerMode(IntEnum):
    LOCAL = 0
    GLOBAL = 1


class Manual25Config:
    def __init__(self):
        self.max_forward_speed = 3
        self.max_sideways_speed = 3
        self.max_angular_speed = np.pi


class Manual25Node(Node):
    def __init__(self):
        super().__init__('manual_25')


    def init(self):
        self.config = self.get_default_config()
        self.mode = ControllerMode.LOCAL
        self.orientation = 0
        self.last_time = 0
        self.delta_t = 0

        # self.max_forward_speed = 1
        self.max_forward_speed = self.config.max_forward_speed
        self.max_sideways_speed = self.config.max_sideways_speed
        # self.max_angular_speed = np.pi/4
        self.max_angular_speed = self.config.max_angular_speed

        self.controller_state = {}

        self.set_device_state(DeviceState.WARMING)
        
        self.get_logger().info("HERE")

        self.controllerSubscriber = self.create_subscription(
            ControllerInput,
            '/autonav/controller_input',
            self.input_callback,
            10
        )
        
        self.motorPublisher = self.create_publisher(
            MotorInput,
            '/autonav/MotorInput',
            10
        )

        self.controllerSubscriber  # prevent unused variable warning


    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))


    def get_default_config(self):
        return Manual25Config()


    def input_callback(self, msg):
        self.new_time = time.clock()
        self.set_device_state(DeviceState.OPERATING)
        self.deserialize_controller_state(msg)
        self.get_logger().info(f"I heard: {str(self.controller_state)}")

        self.changed_controller_mode()
        self.change_system_state()

        # local vs. global toggle
        
        self.delta_t = self.new_time - self.last_time
        if self.mode == ControllerMode.LOCAL:
            self.compose_motorinput_message_local()
        elif self.mode == ControllerMode.GLOBAL:
            self.compose_motorinput_message_global()

        self.last_time = self.new_time

    def deserialize_controller_state(self, msg):
        attributes = [n for n in dir(msg) if not (n.startswith('__') or n.startswith('_'))]
        attributes.remove('SLOT_TYPES')
        attributes.remove('get_fields_and_field_types')
        for attribute in attributes:
            self.controller_state[attribute] = getattr(msg, attribute)


    def normalize(self, input, output_start, output_end, input_start, input_end):
        return output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
    
    
    def changed_controller_mode(self):
        new_controller_mode = ControllerMode.GLOBAL if self.controller_state["btn_select"] == 1.0 else ControllerMode.LOCAL
        self.mode = new_controller_mode


    def change_system_state(self):
        new_system_state = self.system_state
        if self.controller_state['btn_east'] == 1.0:
            new_system_state = SystemState.SHUTDOWN
            
        elif self.controller_state['btn_start'] == 1.0:
            new_system_state = SystemState.MANUAL

        elif self.controller_state['btn_select'] == 1.0:
            new_system_state = SystemState.DISABLED

        self.get_logger().info(f'Setting system state to {new_system_state}')
        self.set_system_state(new_system_state)


    def compose_motorinput_message_local(self):
        forward_velocity = 0.0
        sideways_velocity = 0.0
        angular_velocity = 0.0
        if self.system_state == SystemState.MANUAL:
            forward_velocity = self.normalize(self.controller_state["abs_y"], -self.max_forward_speed, self.max_forward_speed, -1.0, 1.0)
            sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.max_sideways_speed, self.max_sideways_speed, -1.0, 1.0)
            angular_velocity = self.normalize(self.controller_state["abs_z"], -self.max_angular_speed, self.max_angular_speed, -1.0, 1.0)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity
        motor_msg.sideways_velocity = sideways_velocity
        motor_msg.angular_velocity = angular_velocity

        self.motorPublisher.publish(motor_msg)
        self.orientation += angular_velocity * self.delta_t
    

    def compose_motorinput_message_global(self):
        forward_velocity = 0.0
        sideways_velocity = 0.0
        angular_velocity = 0.0
        if self.system_state == SystemState.MANUAL:
            forward_velocity = self.normalize(self.controller_state["abs_y"], -self.max_forward_speed, self.max_forward_speed, -1.0, 1.0)
            sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.max_sideways_speed, self.max_sideways_speed, -1.0, 1.0)
            angular_velocity = self.normalize(self.controller_state["abs_z"], -self.max_angular_speed, self.max_angular_speed, -1.0, 1.0)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity * abs(np.cos(self.orientation)) + sideways_velocity * abs(np.sin(self.orientation))
        motor_msg.sideways_velocity = sideways_velocity * abs(np.cos(self.orientation)) - forward_velocity * abs(np.sin(self.orientation))
        motor_msg.angular_velocity = angular_velocity

        self.motorPublisher.publish(motor_msg)
        self.orientation += angular_velocity * self.delta_t


def main(args=None):
    rclpy.init()
    node = Manual25Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()