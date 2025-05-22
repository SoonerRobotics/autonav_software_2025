#!/usr/bin/env python3

from enum import Enum
from autonav_msgs.msg._motor_input import MotorInput
from autonav_msgs.msg._position import Position
import rclpy
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState
from nav_msgs.msg import OccupancyGrid

class SelfDriveBIGState(Enum):
    IDLE = 0
    MOVE_UNTIL_BARREL = 1
    MOVE_UNTIL_INTERSECTION = 2
    TURN_RIGHT = 3
    TURN_LEFT = 4

class SelfDriveOverwatch(Node):
    def __init__(self):
        super().__init__("self_drive_overwatch")

        self.nominal_speed = 1.4 # 3 mph

        self.current_state = SelfDriveBIGState.IDLE
        self.state_queue = [
            SelfDriveBIGState.MOVE_UNTIL_BARREL,
            SelfDriveBIGState.IDLE
        ]

        self.motor_input = MotorInput()

        self.obstacle_detected = False

    def init(self):
        self.create_subscription(Position, "/autonav/position", self.on_position, 20)
        self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.on_cfg_space, 10)
        self.motorPublisher = self.create_publisher(MotorInput, "/autonav/motor_input", 20)
        self.set_device_state(DeviceState.READY)

    def on_position(self, msg: Position):
        # Only process the position message if the system is in autonomous mode
        if self.system_state != SystemState.AUTONOMOUS:
            return
        
        # Process the position message and update the state
        if self.current_state == SelfDriveBIGState.IDLE:
            self.publish_motion(0.0, 0.0, 0.0)
        elif self.current_state == SelfDriveBIGState.MOVE_UNTIL_BARREL:
            if self.obstacle_detected:
                self.publish_motion(0.0, 0.0, 0.0)

                # Once we have stopped, advance to the next state
                if abs(msg.x_vel) < 0.2:
                    self.advance_state()
            else:
                self.publish_motion(self.nominal_speed, 0.0, 0.0)
        elif self.current_state == SelfDriveBIGState.MOVE_UNTIL_INTERSECTION:
            if self.obstacle_detected:
                self.publish_motion(0.0, 0.0, 0.0)

                # Once we have stopped, advance to the next state
                if abs(msg.x_vel) < 0.2:
                    self.advance_state()
            else:
                self.publish_motion(self.nominal_speed, 0.0, 0.0)
        elif self.current_state == SelfDriveBIGState.TURN_RIGHT:
            # TODO: Implement turn right logic
            pass
        elif self.current_state == SelfDriveBIGState.TURN_LEFT:
            # TODO: Implement turn left logic
            pass
        else:
            self.get_logger().error(f"Unknown state: {self.current_state}")
            self.publish_motion(0.0, 0.0, 0.0)

    def on_cfg_space(self, msg: OccupancyGrid):
        conf_space = msg.data

        # Check for obstacle 3ft ahead
        target_x = 40
        target_y = 59
        
        # Check if any of the target cells in a 3x3 grid are occupied
        for i in range(-1, 2):
            for j in range(-1, 2):
                index = (target_x + i) + (target_y + j) * 80
                if conf_space[index] > 0:
                    self.obstacle_detected = True
                    return
        
        self.obstacle_detected = False
            
    def publish_motion(self, forward_velocity, sideways_velocity, angular_velocity):
        self.motor_input.forward_velocity = forward_velocity
        self.motor_input.sideways_velocity = sideways_velocity
        self.motor_input.angular_velocity = angular_velocity
        self.motorPublisher.publish(self.motor_input)

    def on_system_state_updated(self, old, new):
        if new == SystemState.AUTONOMOUS and self.device_states.get(self.get_name()) == DeviceState.READY:
            self.set_device_state(DeviceState.OPERATING)
            self.begin_sequence_if_idle()

        if new != SystemState.AUTONOMOUS and self.device_states.get(self.get_name()) == DeviceState.OPERATING:
            self.set_device_state(DeviceState.READY)
            self.current_state = SelfDriveBIGState.IDLE

    def begin_sequence_if_idle(self):
        # Check if we have a loaded sequence
        if len(self.state_queue) == 0:
            self.current_state = SelfDriveBIGState.IDLE
            self.get_logger().info("No sequence loaded, returning to IDLE state")
            return
        
        if self.current_state == SelfDriveBIGState.IDLE:
            self.current_state = self.state_queue.pop(0)
            self.get_logger().info(f"Starting sequence: {self.current_state}")
            

    def advance_state(self):
        # Check if we have a loaded sequence
        if len(self.state_queue) == 0:
            self.current_state = SelfDriveBIGState.IDLE
            self.get_logger().info("Sequence complete, returning to IDLE state")
            return
        
        self.current_state = self.state_queue.pop(0)
        self.get_logger().info(f"Advancing to state: {self.current_state}")

 
def main():
    rclpy.init()
    Node.run_node(SelfDriveOverwatch())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
