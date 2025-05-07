#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position, SafetyLights
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
import threading
import math
import rclpy
import time


IS_SOUTH = False
BACK_SPEED = 0.40


class PathResolverConfig:
    def __init__(self):
        self.forward_speed = 1.3
        self.reverse_speed = -0.4
        self.radius_multiplier = 1.2
        self.radius_max = 4.0
        self.radius_start = 0.7
        self.angular_aggressiveness = 2.2
        self.max_angular_speed = 1.3


class PathResolverNode(Node):
    def __init__(self):
        super().__init__("zemlin_path_resolver")
        self.position = Position()
        self.config = PathResolverConfig()

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

        self.resolve_thread = threading.Thread(target=self.resolve)
        self.resolve_thread.daemon = True
        self.resolve_thread.start()
        self.set_device_state(DeviceState.READY)

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
        while True:
            if self.device_states.get(self.get_name()) != DeviceState.OPERATING or self.system_state != SystemState.AUTONOMOUS:
                continue

            cur_pos = (self.position.x, self.position.y)
            lookahead = None
            radius = self.config.radius_start
            while lookahead is None and radius <= self.config.radius_max:
                # self.log(f"Radius: {radius}")
                lookahead = self.purePursuit.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
                radius *= self.config.radius_multiplier

            # inputPacket = MotorInput()
            # inputPacket.forward_velocity = 0.0
            # inputPacket.angular_velocity = 0.0

            # if self.backCount == -1 and (lookahead is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.25):
            #     angle_diff = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
            #     error = self.angle_diff(angle_diff, self.position.theta) / math.pi
            #     forward_speed = self.config.forward_speed * (1 - abs(error)) ** 5
            #     inputPacket.forward_velocity = forward_speed
            #     inputPacket.angular_velocity = self.clamp(error * self.config.angular_aggressiveness, -self.config.max_angular_speed, self.config.max_angular_speed)

            #     if self.status == 0:
            #         self.status = 1
            # else:
            #     if self.backCount == -1:
            #         self.backCount = 8
            #     else:
            #         self.status = 0
            #         self.backCount -= 1

            #     inputPacket.forward_velocity = self.config.reverse_speed
            #     inputPacket.angular_velocity = BACK_SPEED if IS_SOUTH else (-1 * BACK_SPEED)

            # if not self.is_mobility:
            #     inputPacket.forward_velocity = 0.0
            #     inputPacket.angular_velocity = 0.0

            # self.log(f"Forward: {inputPacket.forward_velocity}, Angular: {inputPacket.angular_velocity}")

            # self.motorPublisher.publish(inputPacket)
            
            input = MotorInput()
            input.forward_velocity = 0.0
            input.sideways_velocity = 0.0
            input.angular_velocity = 0.0
            
            # move towards the target and rotate towards the target
            # we are using a swerve drive
            
            if lookahead is not None:
                # forward should be cos(angle_to_pp_goal)
                # sideways should be sin(angle_to_pp_goal)
                
                angle_to_pp_goal = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
                forward_speed = self.config.forward_speed * math.cos(angle_to_pp_goal)
                sideways_speed = -self.config.forward_speed * math.sin(angle_to_pp_goal)
                angular_speed = -self.clamp(self.angle_diff(angle_to_pp_goal, self.position.theta) * self.config.angular_aggressiveness, -self.config.max_angular_speed, self.config.max_angular_speed)
                
                input.forward_velocity = forward_speed
                input.sideways_velocity = sideways_speed
                input.angular_velocity = angular_speed
                
                if self.status == 0:
                    self.status = 1
            else:
                if self.backCount == -1:
                    self.backCount = 8
                else:
                    self.status = 0
                    self.backCount -= 1

                input.forward_velocity = self.config.reverse_speed
                input.angular_velocity = BACK_SPEED if IS_SOUTH else (-1 * BACK_SPEED)
                input.sideways_velocity = BACK_SPEED if IS_SOUTH else (-1 * BACK_SPEED)
                
            if not self.is_mobility:
                # self.log("Not mobility, stopping")
                input.forward_velocity = 0.0
                input.angular_velocity = 0.0
                input.sideways_velocity = 0.0
                
            # self.log(f"Forward: {input.forward_velocity}, Angular: {input.angular_velocity}, Sideways: {input.sideways_velocity}")
            self.motorPublisher.publish(input)
            
            # sleep such that it runs 5 times a second
            time.sleep(0.1)

 
def main():
    rclpy.init()
    rclpy.spin(PathResolverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
