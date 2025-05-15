#!/usr/bin/env python3

import rclpy
import evdev
from evdev import ecodes
import time
from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from std_msgs.msg import String
from autonav_msgs.msg import ControllerInput

JOY_MIN = 0
JOY_MAX = 65535

TRIGGER_MIN = 0
TRIGGER_MAX = 1023

# install the bluez bluetooth drivers for ubuntu:
# sudo apt install bluez
# enable UserspaceHID=true in /etc/bluetooth/input.conf
# robot ip: 192.168.1.79

class ControllerInputConfig():
    def __init__(self):
        self.deadzone = 30.0
        self.timer_period = 0.05

class ControllerInputNode(Node):
    def __init__(self):
        super().__init__('controller_input')
        self.config = ControllerInputConfig()

    def apply_config(self, config: dict):
        self.config.deadzone = config["deadzone"]
        self.config.timer_period = config["timer_period"]

    def on_config_update(self, old_cfg, new_cfg):
        if self.timer is not None:
            self.destroy_timer(self.timer)
        
        self.timer = self.create_timer(self.config.timer_period, self.on_timer_callback)

    def init(self):
        self.publisher = self.create_publisher(ControllerInput, '/autonav/controller_input', 10)
        
        self.controller = self.get_controller()
        self.stick_names = ["ABS_X", "ABS_Y", "ABS_Z", "ABS_RZ"]
        self.dpad_names = ["ABS_HAT0Y", "ABS_HAT0X"]
        self.button_names = ["BTN_NORTH", "BTN_EAST", "BTN_SOUTH", "BTN_WEST"]
        
        self.controller_state = {
            "ABS_BRAKE": 0.0,
            "ABS_GAS": 0.0,
            "ABS_HAT0X": 0.0,
            "ABS_HAT0Y": 0.0,
            "ABS_RZ": 0.0,
            "ABS_X": 0.0,
            "ABS_Y": 0.0,
            "ABS_Z": 0.0,
            "BTN_EAST": 0.0,
            "BTN_MODE": 0.0,
            "BTN_NORTH": 0.0,
            "BTN_SELECT": 0.0,
            "BTN_START": 0.0,
            "BTN_SOUTH": 0.0,
            "BTN_TL": 0.0,
            "BTN_TR": 0.0,
            "BTN_WEST": 0.0,
            "KEY_RECORD": 0.0,
            "wildcard": 0.0
        }

    def get_controller(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        controller = None

        for device in devices:
            if device.name == 'Xbox Wireless Controller':
                controller = evdev.InputDevice(device.path)
                self.get_logger().info(f"assigned controller: \nName: {device.name} \nPath: {device.path} \nBluetooth MAC address: {device.uniq}\n" +
                                       f"\nController state will be published on update and every {self.timer_period_s} seconds")
        
        if controller is not None:
            self.set_device_state(DeviceState.OPERATING)

        return controller
    

    def normalize(self, input, output_start, output_end, input_start, input_end, deadzone_percent):
        output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
        if abs(output) < (deadzone_percent / 100.0):
            return 0.0
        else:
            return output

    def on_timer_callback(self):
        try:
            if self.controller is None:
                if self.get_device_state() == DeviceState.OPERATING:
                    self.set_device_state(DeviceState.READY)

                return

            event = self.controller.read_one()
            if event is None:
                return
            
            if event.type in ecodes.bytype:
                codename = ecodes.bytype[event.type][event.code]
            else:
                codename = "wildcard"

            # We don't care about these
            if ecodes.EV[event.type] == "EV_SYN" or ecodes.EV[event.type] == "EV_MSC":
                return

            value = 0.0
            if ecodes.EV[event.type] == "EV_ABS": # joysticks, dpad, or triggers
                if codename in self.stick_names:
                    value = self.normalize(event.value, -1, 1, JOY_MIN, JOY_MAX, self.config.get("deadzone"))
                elif codename in self.dpad_names:
                    value = float(event.value)
                else: #trigger
                    value = self.normalize(event.value, 0, 1, TRIGGER_MIN, TRIGGER_MAX, self.config.get("deadzone"))
            elif ecodes.EV[event.type] == "EV_KEY": # buttons
                value = float(event.value)

            # codenames for the controller right thumb buttons are an array of possible names, match for one of them
            matches = None
            if type(codename) == list:
                matches = list(set(codename) & set(self.button_names))
                if matches is not None:
                    button_name = self.xbox_swap_N_W(matches[0])
                else:
                    button_name = "wildcard"
            else:
                button_name = codename

            self.update_controller_state(button_name, value)
            msg = self.construct_controller_state_message()
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in controller input: {e}")
            self.set_device_state(DeviceState.ERROR)
            self.controller = None
            self.controller = self.get_controller()
            if self.controller is not None:
                self.get_logger().info("reconnected!")

    def construct_controller_state_message(self):
        message = ControllerInput()

        message.btn_north = self.controller_state["BTN_NORTH"]
        message.btn_east = self.controller_state["BTN_EAST"]
        message.btn_south = self.controller_state["BTN_SOUTH"]
        message.btn_west = self.controller_state["BTN_WEST"]
        message.btn_start = self.controller_state["BTN_START"]
        message.btn_select = self.controller_state["BTN_SELECT"]
        message.btn_mode = self.controller_state["BTN_MODE"]        
        message.btn_tr = self.controller_state["BTN_TR"]        
        message.btn_tl = self.controller_state["BTN_TL"]
        message.key_record = self.controller_state["KEY_RECORD"]
        message.abs_hat0x = self.controller_state["ABS_HAT0X"]   
        message.abs_hat0y = self.controller_state["ABS_HAT0Y"]           
        message.abs_x = self.controller_state["ABS_X"]           
        message.abs_y = self.controller_state["ABS_Y"]
        message.abs_z = self.controller_state["ABS_Z"]
        message.abs_rz = self.controller_state["ABS_RZ"]
        message.abs_gas = self.controller_state["ABS_GAS"]      
        message.abs_brake = self.controller_state["ABS_BRAKE"]
        message.wildcard = self.controller_state["wildcard"]
        
        return message

    def update_controller_state(self, key, value):
        self.controller_state[key] = value

    def xbox_swap_N_W(self, BTN_DIR):
        if BTN_DIR == "BTN_NORTH":
            return "BTN_WEST"
        elif BTN_DIR == "BTN_WEST":
            return "BTN_NORTH"
        else:
            return BTN_DIR
        

def main():
    rclpy.init()
    node = ControllerInputNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

