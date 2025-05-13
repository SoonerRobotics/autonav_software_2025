#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from particlefilter import ParticleFilter
from deadrekt import DeadReckoningFilter
from autonav_msgs.msg import Position
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState
from enum import IntEnum
import rclpy
import math


class FilterType(IntEnum):
    DEAD_RECKONING = 0,
    PARTICLE_FILTER = 1

class FiltersConfig:
    def __init__(self):
        self.filter_type = FilterType.DEAD_RECKONING
        self.degree_offset = 107.0
        self.seed_heading = False
        self.latitude_length = 111086.2
        self.longitude_length = 81978.2

class FiltersNode(Node):
    def __init__(self):
        super().__init__("zemlin_filters")
        self.config = FiltersConfig()

        self.lastIMUReceived = None
        self.firstGps = None

        self.pf = ParticleFilter(self.config.latitude_length, self.config.longitude_length)
        self.reckoning = DeadReckoningFilter()
        
        self.reset()

    def init(self):
        self.create_subscription(GPSFeedback, "/autonav/gps", self.on_gps, 20)
        self.create_subscription(MotorFeedback, "/autonav/motor_feedback", self.onMotorFeedbackReceived, 20)
        self.positionPublisher = self.create_publisher(Position, "/autonav/position", 20)
        self.set_device_state(DeviceState.OPERATING)

    def apply_config(self, config):
        self.config.filter_type = config["filter_type"]
        self.config.degree_offset = config["degree_offset"]
        self.config.seed_heading = config["seed_heading"]
        self.config.latitude_length = config["latitude_length"]
        self.config.longitude_length = config["longitude_length"]

    def on_config_update(self, old_cfg, new_cfg):
        self.pf = ParticleFilter(self.config.latitude_length, self.config.longitude_length)
        self.reckoning = DeadReckoningFilter()

    def get_heading(self, heading: float):
        if heading < 0:
            heading = 360 + -heading
        
        heading += self.config.degree_offset
        return heading

    def reset(self):
        self.reckoning.reset()
        self.pf.init_particles()

    def on_system_state_updated(self, old, new):
        if old != SystemState.AUTONOMOUS and new == SystemState.AUTONOMOUS:
            self.reset()

    def on_mobility_updated(self, old, new):
        if old == False and new == True:
            self.reset()
            
    def on_gps(self, msg: GPSFeedback):
        # if msg.gps_fix == 0 and msg.is_locked == False:
        #     return
        
        if self.firstGps is None:
            self.firstGps = msg

        filterType = self.config.filter_type
        self.pf.gps(msg)
        self.reckoning.gps(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        averages = None
        # if self.get_parameter_or("simulation", "false") == "true":
        averages = self.reckoning.feedback(msg)
        # else:    
        # averages = self.pf.feedback(msg)
            
        if averages is None:
            return
            
        position = Position()
        position.x = averages[0]
        position.y = averages[1]
        position.theta = averages[2]

        # add 180 degrees to the heading (needed for simulator idk why)
        # if self.get_parameter_or("simulation", "false") == "true":
        # position.theta = (position.theta + math.pi) % (2 * math.pi)
        
        # if self.firstGps is not None:
        #     gps_x = self.firstGps.latitude + position.x / self.config.latitude_length
        #     gps_y = self.firstGps.longitude - position.y / self.config.longitude_length
        #     position.latitude = gps_x
        #     position.longitude = gps_y
        
        self.positionPublisher.publish(position)


def main():
    rclpy.init()
    Node.run_node(FiltersNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()