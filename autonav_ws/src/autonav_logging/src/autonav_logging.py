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
import threading

bridge = CvBridge()
camera_topics = [
    # Regular Cameras
    "/autonav/camera/front",
    "/autonav/camera/back",
    "/autonav/camera/left",
    "/autonav/camera/right",

    # Zemlin Code
    "/autonav/cfg_space/expanded/image",
    "/autonav/cfg_space/raw/image",

    # Processed Cameras
    "/autonav/vision/filtered/front",
    "/autonav/vision/filtered/back",
    "/autonav/vision/filtered/left",
    "/autonav/vision/filtered/right",

    # Combined Cameras
    "/autonav/vision/combined/filtered",
    "/autonav/vision/combined/debug",

    # Feelers Debug
    "/autonav/feelers/debug",
]
FPS = 8


class LoggingNode(Node):
    """
    This Node does logs some stuff :)
    """
    
    def __init__(self):
        super().__init__('autonav_logging')
        
        self.file = None
        self.events = []
        self.subs = []

        self.video_writers = {}
        self.QOS = 20
        self.started_logging_at = time.time()
        self.home_dir = os.path.expanduser("~")
        
        # Timer for periodically writing to the file
        self.file_timer = self.create_timer(5, self.write_file)

        # Topic Listeners
        self.deviceStateSub = self.create_subscription(DeviceState, '/autonav/shared/device', self.deviceStateCallback, self.QOS)

        # Create camera subscribers
        for topic in camera_topics:
            subber = self.create_subscription(CompressedImage, topic, lambda msg, topic=topic: self.camera_callback(msg, topic), self.QOS)
            self.subs.append(subber)
        
        # IMU is still TBD
        self.imu_subscriber  = self.create_subscription(IMUFeedback, '/autonav/imu', self.imu_feedback, self.QOS)
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
    
    def create_entry(self):
        self.log("Create_entry", LogLevel.INFO)
        stateFrmt = "autonomous" if self.system_state == 2 else "manual"
        cur_time_date = datetime.now()
        self.started_recording_at = cur_time_date.strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"output.suslog"
        
        self.BASE_PATH = os.path.join(self.home_dir, ".autonav", "logs", stateFrmt, self.started_recording_at)
        self.FILE_PATH = os.path.join(self.BASE_PATH, filename)
        self.log(f"Creating file {self.FILE_PATH}", LogLevel.INFO)
        os.makedirs(self.BASE_PATH, exist_ok=True)
        self.file = open(self.FILE_PATH, "w")

        # create video file in append mode so we can add videos as we write them
        self.video_file = open(os.path.join(self.BASE_PATH, "video_list.txt"), "a")

        # write the first entry, metadata stuff
        self.started_logging_at = time.time()
        event = {
            "system_state": self.system_state.value,
            "mobility": self.mobility,
            "videos": []
        }
        self.append_event("metadata", event)
    
    def normalize_topic(self, topic: str):
        topic = topic.replace("/", "_")
        topic = topic.lower()
        topic = ''.join(e for e in topic if e.isalnum() or e == "_")
        topic = topic.strip("_")

        return topic

    def close_entry(self, state):
        if self.file is None:
            return
        
        self.log("Closing Entry")
        self.write_file()
        self.file.close()
        self.file = None

        # Close the video files and write the video list
        videos = []
        for topic, writer in self.video_writers.items():
            if writer is not None:
                filename = os.path.join(self.BASE_PATH, f"{topic}.avi")
                videos.append(filename)
                writer.release()
        self.video_writers = {}
                
        # Update metadata videos list
        metadata_event = self.events[0]
        metadata_event["event"]["videos"] = videos

        # Sleep for a sec
        time.sleep(1)

        # Stuff
        stateFrmt = "autonomous" if state == 2 else "manual"

        # Zip the entire log folder
        zip_file = os.path.join(self.home_dir, ".autonav", "logs", f"{stateFrmt}_{self.started_recording_at}.zip")
        subprocess.run(["zip", "-r", zip_file, "."], cwd=self.BASE_PATH)
        self.log(f"Zipped logs to {zip_file}")


    def on_system_state_updated(self, old, new):
        if self.file == None and (new == SystemStateEnum.MANUAL or new == SystemStateEnum.AUTONOMOUS):
            self.create_entry()
        elif new != SystemStateEnum.MANUAL and new != SystemStateEnum.AUTONOMOUS and self.file != None:
            self.close_entry(old)       
    
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
            # self.log("File is None, not writing", LogLevel.ERROR)
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


    def on_mobility_updated(self, old, new):
        if self.file == None:
            return
        
        if old != new:
            self.mobility = new
            event = {
                "mobility": new
            }
            self.append_event("mobility", event)

    def camera_callback(self, msg: CompressedImage, topic: str):
        if self.file == None:
            return
        
        topic = self.normalize_topic(topic)
        if topic not in self.video_writers:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            filename = os.path.join(self.BASE_PATH, f"{topic}.avi")
            
            size = (640, 480)
            if "combined" in topic or "feeler" in topic: # it's a combined one, so it's like 1600x1600 or something TODO FIXME
                size = (1600, 1600)

            self.video_writers[topic] = cv2.VideoWriter(filename, fourcc, FPS, size)

        if self.video_writers[topic] is not None:
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.video_writers[topic].write(cv_image)
        
    def deviceStateCallback(self, msg: DeviceState):
        if self.file == None:
            return
        
        self.append_event("device_state", {
            "device": msg.device,
            "state": msg.state
        })
        
    def imu_feedback(self, msg):
        self.append_event("imu", {
            "yaw": msg.yaw,
            "pitch": msg.pitch,
            "roll": msg.roll
        })
    
    def gps_feedback(self, msg):
        self.append_event("gps", {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "num_satellites": msg.num_satellites,
            "gps_fix": msg.gps_fix
        })
    
    def mfeedback_feedback(self, msg):
        self.append_event("motor_feedback", {
            "delta_x": msg.delta_x,
            "delta_y": msg.delta_y,
            "delta_theta": msg.delta_theta,
        })
        
    def minput_feedback(self, msg):
        self.append_event("motor_input", {
            "forward_velocity": msg.forward_velocity,
            "sideways_velocity": msg.sideways_velocity,
            "angular_velocity": msg.angular_velocity
        })
    
    def position_feedback(self, msg: Position):
        self.append_event("position", {
            "x": msg.x,
            "y": msg.y,
            "theta": msg.theta,
            "longitude": msg.longitude,
            "latitude": msg.latitude
        })
        
    def nuc_feedback(self, msg):
        self.append_event("nuc", {
            "timestamp": msg.timestamp,
            "cpu_percentage": msg.cpu_percentage,
            "ram_usage": msg.ram_usage,
            "disk_usage": msg.disk_usage,
            "gpu_usage": msg.gpu_usage
        })
    
    def ultrasonic_feedback(self, msg):
        self.append_event("ultrasonic", {
            "id": msg.id,
            "distance": msg.distance,
        })
        
    def conbus_feedback(self, msg):
        self.append_event("conbus", msg)
    
    def safetylight_feedback(self, msg):
        self.append_event("safetylight", {
            "mode": msg.mode,
            "brightness": msg.brightness,
            "red": msg.red,
            "green": msg.green,
            "blue": msg.blue,
            "blink_period": msg.blink_period
        })
    
    def performance_feedback(self, msg):
        self.append_event("performance", {
            "name": msg.name,
            "elapsed": msg.elapsed,
        })

    def log_callback(self, msg: Log):
        if self.file == None:
            return
        
        self.append_event("log", {
            "timestamp": msg.timestamp,
            "level": msg.level,
            "node": msg.node,
            "message": msg.message,
            "function_caller": msg.function_caller,
            "line_number": msg.line_number
        })

    def particle_feedback(self, msg):
        self.append_event("pf_debug", {
            "x": msg.x,
            "y": msg.y,
            "theta": msg.theta,
            "particles": [{
                "x": p.x,
                "y": p.y,
                "theta": p.theta,
                "weight": p.weight
            } for p in msg.particles]
        })
    
        
def main():
    rclpy.init()
    node = LoggingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()