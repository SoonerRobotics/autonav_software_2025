import pygame
import serial
import can
import struct
import time
from ctypes import Structure, c_uint16, c_uint8, c_bool


pygame.init()
pygame.joystick.init()

# get gamepad
controller = pygame.joystick.Joystick()


# open CAN


forward = controller.get_axis(1)
sideways = controller.get_axis(0)
angular = controller.get_axis(4)


while True:
    pygame.event.get()