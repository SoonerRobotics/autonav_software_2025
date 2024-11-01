#!/usr/bin/env python3

import rclpy
import json
from autonav_msgs.msg import ControllerInput, MotorInput, MotorFeedback
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
        self.odom_fudge_factor = 1


class Manual25Node(Node):
    def __init__(self):
        super().__init__('manual25_node')
        self.write_config(Manual25Config())


    def init(self):
        self.mode = ControllerMode.LOCAL
        self.orientation = 0
        self.last_time = 0
        self.delta_t = 0
        self.new_time = time.time()
        # self.max_forward_speed = 1
        self.max_forward_speed = self.config.max_forward_speed
        self.max_sideways_speed = self.config.max_sideways_speed
        # self.max_angular_speed = np.pi/4
        self.max_angular_speed = self.config.max_angular_speed
        self.odom_fudge_factor = self.config.odom_fudge_factor

        self.controller_state = {}

        self.set_device_state(DeviceState.WARMING)

        self.request_all_configs()

        self.get_logger().info("HERE")

        self.controllerSubscriber = self.create_subscription(
            ControllerInput,
            '/autonav/controller_input',
            self.input_callback,
            10
        )

        self.motorSubscription = self.create_subscription(
            MotorFeedback,
            '/autonav/MotorFeedback',
            self.on_motor_feedback,
            10
        )
        
        self.motorPublisher = self.create_publisher(
            MotorInput,
            '/autonav/MotorInput',
            10
        )

        self.controllerSubscriber  # prevent unused variable warning


    def input_callback(self, msg):
        self.set_device_state(DeviceState.OPERATING)
        self.deserialize_controller_state(msg)
        # self.get_logger().info(f"I heard: {str(self.controller_state)}")

        self.change_controller_mode()
        self.change_system_state()

        self.get_logger().info(f"orientation: {self.orientation}")
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
            self.get_logger().info(f'changed controller mode to {self.mode}')
        elif self.controller_state["btn_south"] == 1.0:
            self.orientation = 0
            self.mode = ControllerMode.LOCAL
            self.get_logger().info(f'changed controller mode to {self.mode}')
            

    def change_system_state(self):
        new_system_state = self.system_state
        if self.controller_state['btn_east'] == 1.0:
            new_system_state = SystemState.SHUTDOWN
            self.get_logger().info(f'Setting system state to {new_system_state}')
            self.set_system_state(new_system_state)
            
        elif self.controller_state['btn_start'] == 1.0:
            new_system_state = SystemState.MANUAL
            self.get_logger().info(f'Setting system state to {new_system_state}')
            self.set_system_state(new_system_state)

        elif self.controller_state['btn_select'] == 1.0:
            new_system_state = SystemState.DISABLED
            self.get_logger().info(f'Setting system state to {new_system_state}')
            self.set_system_state(new_system_state)


    def compose_motorinput_message_local(self):
        forward_velocity = 0.0
        sideways_velocity = 0.0
        angular_velocity = 0.0
        if self.system_state == SystemState.MANUAL:
            forward_velocity = self.normalize(self.controller_state["abs_y"], -self.max_forward_speed, self.max_forward_speed, 1.0, -1.0)
            sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.max_sideways_speed, self.max_sideways_speed, -1.0, 1.0)
            angular_velocity = self.normalize(self.controller_state["abs_z"], -self.max_angular_speed, self.max_angular_speed, 1.0, -1.0)

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
            forward_velocity = self.normalize(self.controller_state["abs_y"], -self.max_forward_speed, self.max_forward_speed, 1.0, -1.0)
            sideways_velocity = self.normalize(self.controller_state["abs_x"], -self.max_sideways_speed, self.max_sideways_speed, -1.0, 1.0)
            angular_velocity = self.normalize(self.controller_state["abs_z"], -self.max_angular_speed, self.max_angular_speed, 1.0, -1.0)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity * np.cos(self.orientation) + sideways_velocity * np.sin(self.orientation)
        motor_msg.sideways_velocity = -1 * sideways_velocity * np.cos(self.orientation) + forward_velocity * np.sin(self.orientation)
        motor_msg.angular_velocity = angular_velocity

        self.motorPublisher.publish(motor_msg)


    def on_motor_feedback(self, msg:MotorFeedback):
        delta_theta = msg.delta_theta * self.odom_fudge_factor
        self.orientation += delta_theta

def main(args=None):
    rclpy.init()
    node = Manual25Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()