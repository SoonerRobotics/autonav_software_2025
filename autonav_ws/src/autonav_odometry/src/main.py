#!/usr/bin/env python3
# filters.py from last year for the most part

import json
from types import SimpleNamespace
from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from autonav_shared.types import DeviceState, SystemState, LogLevel # i am goig to beat pylance with a rock
from particlefilter import ParticleFilter
from scr_msgs.msg import SystemState
from autonav_shared.node import Node
from enum import IntEnum
import rclpy

class FilterType(IntEnum):
    PARTICLE_FILTER = 1
    # ADD NEW FILTERS HERE

class FiltersNodeConfig:
    def __init__(self):
        self.filter_type = FilterType.PARTICLE_FILTER

class FiltersNode(Node):
    def __init__(self):
        super().__init__("autonav_filters")
        # init gps and position
        self.first_gps = None
        self.last_gps = None
        self.latitude_length = self.declare_parameter("latitude_length", 111086.2).get_parameter_value().double_value
        self.longitude_length = self.declare_parameter("longitude_length", 81978.2).get_parameter_value().double_value
        # init filters
        # IF WE IMPLEMENT MORE FILTERS ADD THEM HERE (AND ANYWHERE THAT COMES UP WHEN YOU CTRL-F "NEW FILTERS")!
        self.pf = ParticleFilter(self.latitude_length, self.longitude_length)
        self.config = FiltersNodeConfig()
        self.onReset()

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))
        # reinit filters after getting new config
        self.onReset()

    def init(self):
        # [melee announcer voice] ready...
        self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.position_publisher = self.create_publisher(Position, "/autonav/position", 20)
        # go!
        self.set_device_state(DeviceState.OPERATING)

    def onReset(self):
        self.first_gps = None
        self.last_gps = None
        self.pf.init_particles()
        # INIT NEW FILTERS HERE

    def system_state_transition(self, old: SystemState, updated: SystemState):
        # reinit filters when robot changes modes
        if old.state != SystemState.AUTONOMOUS and updated.state == SystemState.AUTONOMOUS:
            self.onReset()

        if old.state != SystemState.MANUAL and updated.state == SystemState.MANUAL:
            self.onReset()
        # or gets going
        if old.mobility == False and updated.mobility == True:
            self.onReset()

    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return

        if self.first_gps is None:
            self.first_gps = msg

        self.last_gps = msg
        # USE NEW FILTERS HERE
        if self.config.filter_type == FilterType.PARTICLE_FILTER:
            self.pf.gps(msg)
        else:
            self.log(f"{self.config.filter_type} isn't a valid FilterType! Did you implement a new filter and forget to use it?", LogLevel.FATAL)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        averages = None
        # USE NEW FILTERS HERE
        if self.config.filter_type == FilterType.PARTICLE_FILTER:
            averages = self.pf.feedback(msg)
        else:
            self.log(f"{self.config.filter_type} isn't a valid FilterType! Did you implement a new filter and forget to use it?", LogLevel.FATAL)

        if averages is None:
            return
        # unpack what the filter just returned
        position = Position()
        position.x = averages[0]
        position.y = averages[1]
        position.theta = averages[2]

        if self.first_gps is not None:
            gps_x = self.first_gps.latitude + position.x / self.latitude_length
            gps_y = self.first_gps.longitude - position.y / self.longitude_length
            position.latitude = gps_x
            position.longitude = gps_y

        self.position_publisher.publish(position)

def main():
    rclpy.init()
    node = FiltersNode()
    Node.run_node(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
