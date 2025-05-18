#!/usr/bin/env python3

import rclpy
import yaml
from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState
from autonav_shared.types import SystemState as SystemStateEnum
# from autonav_shared.types import LogLevel, DeviceState, SystemState

from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import *
from cv_bridge import CvBridge

from datetime import datetime
import os
import time
import json

import subprocess
import cv2

bridge = CvBridge()

class LogConfig:
    def __init__(self):
        self.record_imu = True
        self.record_gps = True
        self.record_motor = True
        self.record_position = True
        self.record_nuc = True
        self.record_ultrasonic = True
        self.record_conbus = True
        self.record_safetylights = True
        self.record_performance = True
        self.record_pf_debug = True

class LoggingNode(Node):
    """
    This Node does logs some stuff :)
    """
    
    def __init__(self):
        super().__init__('autonav_logging')
        
        self.file = None
        self.events = []
        self.QOS = 10
        self.started_logging_at = time.time()
        self.home_dir = os.path.expanduser("~")
        self.config = LogConfig()
        
        # Timer for periodically writing to the file
        self.file_timer = self.create_timer(5, self.write_file)

        # Topic Listeners
        self.deviceStateSub = self.create_subscription(DeviceState, '/autonav/shared/device', self.deviceStateCallback, self.QOS)
        
        # IMU is still TBD
        self.imu_subscriber  = self.create_subscription(IMUData, '/autonav/imu', self.imu_feedback, self.QOS)
        self.gps_subscriber = self.create_subscription(GPSFeedback, '/autonav/gps', self.gps_feedback, self.QOS)
        self.input_subscriber = self.create_subscription(MotorInput, '/autonav/motor_input', self.minput_feedback, self.QOS)
        self.feedback_subscriber = self.create_subscription(MotorFeedback, '/autonav/motor_feedback', self.mfeedback_feedback, self.QOS)
        self.position_subscriber = self.create_subscription(Position, '/autonav/position', self.position_feedback, self.QOS)
        self.nuc_subscriber = self.create_subscription(NUCStatistics, '/autonav/statistics', self.nuc_feedback, self.QOS)
        self.ultrasonics_subscriber = self.create_subscription(Ultrasonic, '/autonav/ultrasonics', self.ultrasonic_feedback, self.QOS)
        self.conbus_subscriber = self.create_subscription(Conbus, '/autonav/CONbus', self.conbus_feedback, self.QOS)
        self.safetylight_subscriber = self.create_subscription(SafetyLights, '/autonav/safety_lights', self.safetylight_feedback, self.QOS)
        self.performance_subscriber = self.create_subscription(Performance, '/autonav/performance', self.performance_feedback, self.QOS)
        self.particle_subscriber = self.create_subscription(ParticleFilterDebug, '/autonav/particle_debug', self.particle_feedback, self.QOS)
        self.log_sub = self.create_subscription(Log, "/autonav/shared/log", self.log_callback, self.QOS)

    def apply_config(self, config):
        self.config.record_imu = config["record_imu"]
        self.config.record_gps = config["record_gps"]
        self.config.record_motor = config["record_motor"]
        self.config.record_position = config["record_position"]
        self.config.record_nuc = config["record_nuc"]
        self.config.record_ultrasonic = config["record_ultrasonic"]
        self.config.record_conbus = config["record_conbus"]
        self.config.record_safetylights = config["record_safetylights"]
        self.config.record_performance = config["record_performance"]
    
    def create_entry(self):
        self.log("Create_entry", LogLevel.INFO)
        stateFrmt = "autonomous" if self.system_state == 1 else "manual"
        cur_time_date = datetime.now()
        cur_time = cur_time_date.strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"{stateFrmt}_{cur_time}.suslog"
        
        BASE_PATH = os.path.join(self.home_dir, ".autonav", "logs")
        FILE_PATH = os.path.join(BASE_PATH, filename)
        self.log(f"Creating file {FILE_PATH}", LogLevel.INFO)
        os.makedirs(BASE_PATH, exist_ok=True)
        self.file = open(FILE_PATH, "w")

        # write the first entry, metadata stuff
        self.started_logging_at = time.time()
        event = {
            "system_state": self.system_state.value,
            "mobility": self.mobility
        }
        self.append_event("metadata", event)
    
    def close_entry(self):
        if self.file is None:
            return
        
        self.get_logger().info("Closing Entry")
        self.write_file()
        self.file.close()
        self.file = None

    def on_system_state_updated(self, old, new):
        if self.file == None and (new == SystemStateEnum.MANUAL or new == SystemStateEnum.AUTONOMOUS):
            self.create_entry()
        elif new != SystemStateEnum.MANUAL and new != SystemStateEnum.AUTONOMOUS and self.file != None:
            self.close_entry()       
    
    def append_event(self, event_type: str, event):
        if self.file == None:
            return
        
        msg = {
            "timestamp": time.time() - self.started_logging_at,
            "type": event_type,
            "event": event
        }
        self.events.append(msg)

    def write_file(self):
        if self.file == None:
            self.log("File is None, not writing", LogLevel.ERROR)
            return
        
        events_cpy = []
        for event in self.events:
            event_cpy = event.copy()
            try:
                json.dumps(event_cpy["event"])
            except TypeError:
                event_cpy["event"] = yaml.load(str(event["event"]), Loader=yaml.Loader)
            events_cpy.append(event_cpy)

        json_str = json.dumps(events_cpy)
        
        # clear what we have written so far and write the new stuff
        self.file.seek(0)
        self.file.truncate()
        self.file.write(json_str)
        
    def deviceStateCallback(self, msg):
        if self.file == None:
            return
        
        self.append_event("device_state", msg)
        
    def imu_feedback(self, msg):
        if not self.config.record_imu:
            return
        
        self.append_event("imu", msg)
    
    def gps_feedback(self, msg):
        if not self.config.record_gps:
            return
        
        self.append_event("gps", msg)
    
    def mfeedback_feedback(self, msg):
        if not self.config.record_motor:
            return
        
        self.append_event("motor_feedback", msg)
        
    def minput_feedback(self, msg):
        if not self.config.record_motor:
            return
        
        self.append_event("motor_input", msg)
    
    def position_feedback(self, msg: Position):
        if not self.config.record_position:
            return
        
        self.append_event("position", {
            "x": msg.x,
            "y": msg.y,
            "theta": msg.theta,
            "longitude": msg.longitude,
            "latitude": msg.latitude
        })
        
    def nuc_feedback(self, msg):
        if not self.config.record_nuc:
            return
        
        self.append_event("nuc", msg)
    
    def ultrasonic_feedback(self, msg):
        if not self.config.record_ultrasonic:
            return
        
        self.append_event("ultrasonic", msg)
        
    def conbus_feedback(self, msg):
        if not self.config.record_conbus:
            return
        
        self.append_event("conbus", msg)
    
    def safetylight_feedback(self, msg):
        if not self.config.record_safetylights:
            return
        
        self.append_event("safetylight", msg)
    
    def performance_feedback(self, msg):
        if not self.config.record_performance:
            return
        
        self.append_event("performance", msg)

    def log_callback(self, msg: Log):
        if self.file == None:
            return
        
        self.append_event("log", msg)

    def particle_feedback(self, msg):
        if not self.config.record_pf_debug:
            return
        
        self.append_event("pf_debug", msg)
    
        
def main():
    rclpy.init()
    node = LoggingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()