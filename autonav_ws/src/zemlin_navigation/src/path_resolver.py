#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position, SafetyLights, AudibleFeedback
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
import threading
import math
import rclpy
import time
import os


IS_SOUTH = False
BACK_SPEED = 0.40


class PathResolverConfig:
    def __init__(self):
        self.forward_speed = 3.8
        self.reverse_speed = -1.1
        self.radius_multiplier = 1.2
        self.radius_max = 4.0
        self.radius_start = 0.7
        self.angular_aggressiveness = 14
        self.max_angular_speed = 4
        self.backup_sound = '~/autonav_software_2025/music/truck.mp3'

class PathResolverNode(Node):
    def __init__(self):
        super().__init__("zemlin_path_resolver")
        self.position = Position()
        self.config = PathResolverConfig()

    def apply_config(self, config):
        self.config.forward_speed = config["forward_speed"]
        self.config.reverse_speed = config["reverse_speed"]
        self.config.radius_multiplier = config["radius_multiplier"]
        self.config.radius_max = config["radius_max"]
        self.config.radius_start = config["radius_start"]
        self.config.angular_aggressiveness = config["angular_aggressiveness"]
        self.config.max_angular_speed = config["max_angular_speed"]

    def init(self):
        self.purePursuit = PurePursuit()
        self.backCount = -1
        self.status = -1
        self.pathSubscriber = self.create_subscription(
            Path, "/autonav/path", self.on_path_received, 20)
        self.positionSubscriber = self.create_subscription(
            Position, "/autonav/position", self.on_position_received, 20)
        self.motorPublisher = self.create_publisher(
            MotorInput, "/autonav/motor_input", 20)
        self.safetyLightsPublisher = self.create_publisher(
            SafetyLights, "/autonav/SafetyLights", 20)

        self.tick_timer = self.create_timer(0.05, self.resolve)
        self.set_device_state(DeviceState.READY)

        self.audibleFeedbackPublisher = self.create_publisher(
                AudibleFeedback,
                '/autonav/audible_feedback',
                10
        )


    def onReset(self):
        self.backCount = -1


    def on_system_state_updated(self, old, new):
        if new == SystemState.AUTONOMOUS and self.device_states.get(self.get_name()) == DeviceState.READY:
            self.set_device_state(DeviceState.OPERATING)

        if new != SystemState.AUTONOMOUS and self.device_states.get(self.get_name()) == DeviceState.OPERATING:
            self.set_device_state(DeviceState.READY)
            inputPacket = MotorInput()
            inputPacket.forward_velocity = 0.0
            inputPacket.angular_velocity = 0.0
            self.motorPublisher.publish(inputPacket)

    def on_position_received(self, msg):
        self.position = msg
        self.position.x = 0.0
        self.position.y = 0.0
        self.position.theta = 0.0

    def angle_diff(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta

    def on_path_received(self, msg: Path):
        self.points = [x.pose.position for x in msg.poses]
        self.purePursuit.set_points([(point.x, point.y)
                                    for point in self.points])

    def clamp(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        return value

    def resolve(self):
        if self.system_state != SystemState.AUTONOMOUS:
            return

        self.perf_start("path_resolve")

        cur_pos = (self.position.x, self.position.y)
        lookahead = None
        radius = self.config.radius_start
        while lookahead is None and radius <= self.config.radius_max:
            # self.log(f"Radius: {radius}")
            lookahead = self.purePursuit.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
            radius *= self.config.radius_multiplier

        inputPacket = MotorInput()
        inputPacket.forward_velocity = 0.0
        inputPacket.sideways_velocity = 0.0
        inputPacket.angular_velocity = 0.0
        
        if not self.is_mobility():
            self.perf_stop("path_resolve")
            self.motorPublisher.publish(inputPacket)
            return

        if self.backCount == -1 and (lookahead is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.25):
            angle_diff = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
            error = self.angle_diff(angle_diff, self.position.theta) / math.pi
            forward_speed = self.config.forward_speed * (1 - abs(error)) ** 5
            inputPacket.forward_velocity = forward_speed
            inputPacket.angular_velocity = self.clamp(error * self.config.angular_aggressiveness, -self.config.max_angular_speed, self.config.max_angular_speed)
            # inputPacket.angular_velocity += 0.1 if self.angul
            # audible_feedback = AudibleFeedback()
            # audible_feedback.filename = os.path.expanduser(self.config.backup_sound)
            # audible_feedback.main_track = False
            # self.audibleFeedbackPublisher.publish(audible_feedback)

            if self.status == 0:
                self.status = 1
        else:
            if self.backCount == -1:
                self.backCount = 4
                # TODO: Push safety lights
            else:
                self.status = 0
                self.backCount -= 1

            inputPacket.forward_velocity = self.config.reverse_speed
            inputPacket.angular_velocity = BACK_SPEED

        self.motorPublisher.publish(inputPacket)
            
        self.perf_stop("path_resolve")

 
def main():
    rclpy.init()
    Node.run_node(PathResolverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
