import pygame
import serial
import can
import struct
import time
from ctypes import Structure, c_uint16, c_uint8, c_bool

SCALE = 1000

pygame.init()
pygame.joystick.init()

timer = pygame.time.Clock()

# get gamepad
controller = pygame.joystick.Joystick(0)
controller.init()

# open CAN
#FIXME bitrate is 125 kbps according to the spec, but rn Micheal's firmware is the same as last year (i.e. 100 kbps)
can_net = can.ThreadSafeBus(bustype="slcan", channel="/dev/ttyACM0", bitrate=100_000)

# class MotorInputPacket(Structure):
#     _fields_ = [
#         ("forward_velocity", c_uint16, 16),
#         ("sideways_velocity", c_uint16, 16),
#         ("angular_velocity", c_uint16, 16),
#     ]

try:
    while True:
        for event in pygame.event.get():
            pass #TODO FIXME ??? handle button presses???

        forward = controller.get_axis(1)
        sideways = controller.get_axis(0)
        angular = controller.get_axis(2)

        # motor_input_packer = MotorInputPacket()
        # motor_input_packer.forward_velocity = int(forward * SCALE)
        # motor_input_packer.sideways_velocity = int(sideways * SCALE)
        # motor_input_packer.angular_velocity = int(angular * SCALE)

        print(f"{forward:.2f} | {sideways:.2f} | {angular:.2f}")

        # data = bytes(motor_input_packer)
        data = struct.pack("<hhh", int(forward * SCALE), int(sideways * SCALE), int(angular * SCALE))
        can_msg = can.Message(arbitration_id=10, data=data)
        
        try:
            can_net.send(can_msg)
        except AttributeError:
            print("Error writing to CAN",)
        except can.CanError:
            print("2 Error writing to CAN")

        try:
            msg = can_net.recv(timeout=0.01)
            if msg is not None:
                print(msg.arbitration_id)
        except can.CanError:
            print("CAN ERROR!!! IN RECV!!!")
        
        timer.tick(100)
except Exception as e:
    print(e)
    can_net.shutdown()
    time.sleep(1)
    pygame.quit()