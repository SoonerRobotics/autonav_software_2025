#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
import copy
from heapq import heappush, heappop
import numpy as np
import cv2
import cv_bridge
import time
from enum import Enum

class MoverState(Enum):
    IDLE = 0 # not moving
    FORWARD = 1 # moving forward
    RIGHT = 2 # moving right
    ZERO = 3 # moving to zero (backwards and left)


DISTANCE = 2 # distance to move
DISTANCE_THRESHOLD = 0.05 # distance to target before changing state
SPEED = 1 # speed of the robot in m/s


class MoverNode(Node):
    def __init__(self):
        super().__init__("zemlin_mover")

        self.position = Position()
        self.target = Position()
        self.state = MoverState.IDLE

    def init(self):
        self.positionSubscriber = self.create_subscription(Position, "/autonav/position", self.on_position_received, 20)
        self.motorPublisher = self.create_publisher(MotorInput, "/autonav/motor_input", 20)
        self.timer = self.create_timer(0.05, self.on_timer)

        self.set_device_state(DeviceState.READY)

    def on_system_state_updated(self, old, new):
        if new == SystemState.AUTONOMOUS and old != SystemState.AUTONOMOUS:
            self.push_safety_lights(255, 255, 255, 1, 0)
            self.state = MoverState.FORWARD
            self.target = copy.deepcopy(self.position)
            self.target.x += DISTANCE
            self.set_device_state(DeviceState.OPERATING)

        if new != SystemState.AUTONOMOUS and old == SystemState.AUTONOMOUS:
            self.push_safety_lights(255, 255, 255, 0,  0)
            self.state = MoverState.IDLE
            self.set_device_state(DeviceState.READY)

            # push 0 velocity to stop the robot
            inputPacket = MotorInput()
            inputPacket.forward_velocity = 0.0
            inputPacket.sideways_velocity = 0.0
            inputPacket.angular_velocity = 0.0
            self.motorPublisher.publish(inputPacket)
            self.state = MoverState.IDLE

    def on_mobility_updated(self, old, new):
        if new == True and old == False:
            pass

        if new == False and old == True:
            # stop the robot
            inputPacket = MotorInput()
            inputPacket.forward_velocity = 0.0
            inputPacket.sideways_velocity = 0.0
            inputPacket.angular_velocity = 0.0
            self.motorPublisher.publish(inputPacket)
            self.state = MoverState.IDLE

    def on_position_received(self, msg):
        self.position = msg

    def on_timer(self):
        if self.state == MoverState.IDLE:
            return
        
        # calculate the distance to the target
        dx = self.target.x - self.position.x
        dy = self.target.y - self.position.y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < DISTANCE_THRESHOLD:
            # State Machine: FORWARD -> RIGHT, RIGHT -> ZERO, ZERO -> IDLE
            if self.state == MoverState.FORWARD:
                self.push_safety_lights(0, 0, 255, 1, 2)
                self.push_safety_lights(255, 255, 255, 1, 0)
                self.state = MoverState.RIGHT
                self.target.x = self.position.x
                self.target.y = self.position.y - DISTANCE
            elif self.state == MoverState.RIGHT:
                self.push_safety_lights(0, 255, 0, 1, 2)
                self.push_safety_lights(255, 255, 255, 1, 0)
                self.state = MoverState.ZERO
                self.target.x = self.position.x - DISTANCE
                self.target.y = self.position.y + DISTANCE
            elif self.state == MoverState.ZERO:
                self.push_safety_lights(255, 255, 255, 0, 0)
                self.state = MoverState.IDLE
                self.target.x = self.position.x
                self.target.y = self.position.y

                # stop the robot
                inputPacket = MotorInput()
                inputPacket.forward_velocity = 0.0
                inputPacket.sideways_velocity = 0.0
                inputPacket.angular_velocity = 0.0
                self.motorPublisher.publish(inputPacket)
                return
        
        # calculate the angle to the target
        angle = math.atan2(dy, dx)
        angle_diff = self.angle_diff(angle, self.position.theta)
        # calculate the motor input (forward and sideways, ignore rotation for now)
        inputPacket = MotorInput()
        inputPacket.forward_velocity = SPEED * math.cos(angle_diff)
        inputPacket.sideways_velocity = SPEED * math.sin(angle_diff)
        # inputPacket.angular_velocity = 0.0

        # rotate towards the target
        # if abs(angle_diff) > math.radians(5):
        #     inputPacket.angular_velocity = -angle_diff * 0.5
        # else:
        inputPacket.angular_velocity = 0.0
            
        # publish the motor input
        self.motorPublisher.publish(inputPacket)

    def angle_diff(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta
                


def main():
    rclpy.init()
    Node.run_node(MoverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()