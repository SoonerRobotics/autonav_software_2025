#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position, ParticleFilterDebug
from particlefilter import ParticleFilter
from deadrekt import DeadReckoningFilter
from bearing import BearingFilter
from autonav_msgs.msg import Position
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState
from enum import IntEnum
import rclpy
import math


class FilterType(IntEnum):
    DEAD_RECKONING = 0,
    PARTICLE_FILTER = 1,
    BEARING_FILTER = 2

class FiltersConfig:
    def __init__(self):
        self.filter_type = FilterType.BEARING_FILTER
        self.latitude_length = 110944.2
        self.longitude_length = 91065.46

class FiltersNode(Node):
    def __init__(self):
        super().__init__("zemlin_filters")
        self.config = FiltersConfig()

        self.lastIMUReceived = None
        self.firstGps = None

        self.pf = ParticleFilter(self.config.latitude_length, self.config.longitude_length)
        self.bf = BearingFilter(self.config.latitude_length, self.config.longitude_length)
        self.reckoning = DeadReckoningFilter()
        
        self.reset()

    def init(self):
        self.create_subscription(GPSFeedback, "/autonav/gps", self.on_gps, 20)
        self.create_subscription(MotorFeedback, "/autonav/motor_feedback", self.onMotorFeedbackReceived, 20)
        self.create_timer(1, self.on_publish_debug)
        self.positionPublisher = self.create_publisher(Position, "/autonav/position", 20)
        self.particleDebugPublisher = self.create_publisher(ParticleFilterDebug, "/autonav/particle_debug", 20)
        self.set_device_state(DeviceState.OPERATING)

    def apply_config(self, config):
        self.config.filter_type = config["filter_type"]
        self.config.latitude_length = config["latitude_length"]
        self.config.longitude_length = config["longitude_length"]

    def on_config_update(self, old_cfg, new_cfg):
        self.pf = ParticleFilter(self.config.latitude_length, self.config.longitude_length)
        self.reckoning = DeadReckoningFilter()

    def reset(self):
        self.reckoning.reset()
        self.bf.reset()
        self.pf.init_particles()

    def on_system_state_updated(self, old, new):
        if old != SystemState.AUTONOMOUS and new == SystemState.AUTONOMOUS:
            self.reset()
            
        if old != SystemState.MANUAL and new == SystemState.MANUAL:
            self.reset()

    def on_mobility_updated(self, old, new):
        if old == False and new == True:
            self.reset()
            
    def on_gps(self, msg: GPSFeedback):
        # if msg.gps_fix == 0 and msg.is_locked == False:
        #     return
        
        if self.firstGps is None:
            self.firstGps = msg

        # Technically we should not run both and we should only run the one that is active
        self.pf.gps(msg)
        self.reckoning.gps(msg)
        
        if self.config.filter_type == FilterType.BEARING_FILTER:
            x, y, theta = self.bf.gps(msg)
            if theta == None:
                return
            
            position = Position()
            position.x = x
            position.y = y
            position.theta = (2 * math.pi) - theta
            position.latitude = msg.latitude
            position.longitude = msg.longitude
            self.positionPublisher.publish(position)

    def on_publish_debug(self):
        if self.config.filter_type == FilterType.PARTICLE_FILTER:
            msg = self.pf.get_particle_feedback()
            if msg is not None:
                self.particleDebugPublisher.publish(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        averages = None
        if self.config.filter_type == FilterType.DEAD_RECKONING:
            averages = self.reckoning.feedback(msg)
        elif self.config.filter_type == FilterType.PARTICLE_FILTER:
            averages = self.pf.feedback(msg)
        else:
            return
            
        if averages is None:
            return
            
        position = Position()
        position.x = averages[0]
        position.y = averages[1]
        position.theta = averages[2]

        # self.log(f"Position: {position.x}, {position.y}, {position.theta}")
        
        if self.firstGps is not None:
            gps_x = self.firstGps.latitude + position.x / self.config.latitude_length
            gps_y = self.firstGps.longitude - position.y / self.config.longitude_length
            position.latitude = gps_x
            position.longitude = gps_y
        
        self.positionPublisher.publish(position)


def main():
    rclpy.init()
    Node.run_node(FiltersNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()