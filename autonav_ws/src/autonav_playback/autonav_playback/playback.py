import rclpy
from rclpy.node import Node

from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import *
from cv_bridge import CvBridge

from datetime import datetime
import os
import time

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

class playback(Node):
    """
    This Node does some stuff :)
    """
    
    def __init__(self):
        super().__init__('autonav_playback')
        
        self.file = None
        
        self.QOS = 10
        self.home_dir = os.path.expanduser("~")
        self.system_state = 0
        self.config = LogConfig()
        
        # Topic Listeners
        self.systemStateSub = self.create_subscription(SystemState, 'autonav/shared/system', self.systemStateCallback, self.QOS)
        self.deviceStateSub = self.create_subscription(DeviceState, 'autonav/shared/device', self.deviceStateCallback, self.QOS)
        
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

        
        
        self.FRAMERATE = 30
        # Raw Camera Record Settings
        self.record_command = ['ffmpeg','-y', '-loglevel', 'error', '-f', 'image2pipe', '-vcodec', 'mjpeg', '-framerate', str(self.FRAMERATE), '-s', '640x480', '-i', '-']
        
        # Raw Camera Topics
        self.camera1 = self.create_subscription(CompressedImage, 'autonav/camera/left', lambda msg: self.cameraCallback(msg, 'left'), self.QOS)
        self.camera2 = self.create_subscription(CompressedImage, 'autonav/camera/right', lambda msg: self.cameraCallback(msg, 'right'), self.QOS)
        self.camera3 = self.create_subscription(CompressedImage, 'autonav/camera/front', lambda msg: self.cameraCallback(msg, 'front'), self.QOS)
        self.camera4 = self.create_subscription(CompressedImage, 'autonav/camera/back', lambda msg: self.cameraCallback(msg, 'back'), self.QOS)
        # Additional Video Feeds
        self.combined = self.create_subscription(CompressedImage, '/autonav/vision/combined/filtered', lambda msg: self.cameraCallback(msg, 'combined'), self.QOS)
        self.feelers = self.create_subscription(CompressedImage, '/autonav/feelers/debug', lambda msg: self.cameraCallback(msg, 'feelers'), self.QOS)
        
        # FFmpeg Process Dict - Stores the ffmpeg pipes
        self.process_dict = {'left':None, 'right':None, 'front':None, 'back':None, 'combined':None, 'feelers':None}
        # Tracks pipe status
        self.closed_dict = {'left':False, 'right':False, 'front':False, 'back':False, 'combined':False, 'feelers':None}
        
        # Video Buffer List | Generally cutoff is equal to number of frames, in this case 15 frames ~ 1 sec of footage
        self.buffer_dict = {'left':[], 'right':[], 'front':[], 'back':[], 'combined':[], 'feelers':[]}
        # Time is money
        t = time.time()
        self.base_time_dict = {'left':t, 'right':t, 'front':t, 'back':t, 'combined':t, 'feelers':t}
        del t
        
        # Video Stream Performance Tracking
        # List of times in seconds to record every frame
        self.performance_dict = {'left':[], 'right':[], 'front':[], 'back':[], 'combined':[], 'feelers':[]}
        
        
    
    def cameraCallback(self, msg, id):
        path = os.path.join(self.home_dir, "Documents", "AutoNav", "Vids")
        os.makedirs(path, exist_ok=True)
        
        def new_process(id: str):
            print('Opening new Proccess')
            
            # Adjust the command output filename
            c = self.record_command
            c.append(path+'/'+id+'.mp4')
            self.process_dict[id] = subprocess.Popen(c, stdin=subprocess.PIPE)
            
            # Start tracking time as soon as pipe opens
            self.buffer_dict[id].append(msg)
            self.base_time_dict[id] = time.time()
        
        def check_buffer(id: str):
            # Check the buffer to see if it has a whole second of frames
            if len(self.buffer_dict[id]) == 30:
                frame_time = self.get_elapsed_seconds(self.base_time_dict[id])
                self.base_time_dict[id] = time.time()
                self.performance_dict[id].append(frame_time)
                # Then Process the whole buffer
                self.process_frame_buffer(self.buffer_dict[id], self.process_dict[id])
                self.print_performance(self.performance_dict[id])
                self.buffer_dict[id] = []
            self.buffer_dict[id].append(msg)
        
        if id == 'left' and self.process_dict["left"] == None and not self.closed_dict['left']:
            new_process('left')
        elif id == 'right' and self.process_dict["right"] == None and not self.closed_dict['right']:
            new_process('right')
        elif id == "front" and self.process_dict["front"] == None and not self.closed_dict['front']:
            new_process('front')
        elif id == "back" and self.process_dict["back"] == None and not self.closed_dict['back']:
            new_process('back')
        elif id == "combined" and self.process_dict["combined"] == None and not self.closed_dict['combined']:
            new_process('combined')
        elif id == "feelers" and self.process_dict["feelers"] == None and not self.closed_dict['feelers']:
            new_process('feelers')
        
        if id == 'left' and not self.closed_dict['left']:
            check_buffer('left')
        elif id == 'right' and not self.closed_dict['right']:
            check_buffer('right')
        elif id == 'front' and not self.closed_dict['front']:
            check_buffer('front')
        elif id == 'back' and not self.closed_dict['back']:
            check_buffer('back')
        elif id == 'combined' and not self.closed_dict['combined']:
            check_buffer('combined')
        elif id == 'feelers' and not self.closed_dict['feelers']:
            check_buffer('feelers')
    
    def process_img(self, msg, process):
        image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        success, encoded_image = cv2.imencode('.jpg', image)
        if success:
            process.stdin.write(encoded_image.tobytes())
        else:
            print("Image Encoding Failed")
    
    def print_performance(self, performance):
        bad = max(performance)
        good = min(performance)
        mean = sum(performance) / len(performance)
        print(f"Complete! Longest Frame: {bad} | Quickest Frame: {good}")
        print(f"Average write time of {mean}")
    
    # This whole method is cancer but bear with me
    def process_frame_buffer(self, buffer, process):
        print("About to process a whole buffer")
        for img in buffer:
            self.process_img(img, process)
    
    def get_elapsed_seconds(self, base: float):
        return time.time() - base
    
    def close_recording(self):
        self.get_logger().info('Closing Recordings')
        if not self.process_dict['left'] == None:
            self.process_dict['left'].stdin.close()
            self.process_dict['left'].wait()
        if not self.process_dict['right'] == None:
            self.process_dict['right'].stdin.close()
            self.process_dict['right'].wait()
        if not self.process_dict['front'] == None:
            self.process_dict['front'].stdin.close()
            self.process_dict['front'].wait()
        if not self.process_dict['back'] == None:
            self.process_dict['back'].stdin.close()
            self.process_dict['back'].wait()
        if not self.process_dict['combined'] == None:
            self.process_dict['combined'].stdin.close()
            self.process_dict['combined'].wait()
        if not self.process_dict['feelers'] == None:
            self.process_dict['feelers'].stdin.close()
            self.process_dict['feelers'].wait()
        
        self.process_dict['left'] = None
        self.process_dict['right'] = None
        self.process_dict['front'] = None
        self.process_dict['back'] = None
        self.process_dict['combined'] = None
        self.process_dict['feelers'] = None
        
        self.closed_dict['left'] = True
        self.closed_dict['right'] = True
        self.closed_dict['front'] = True
        self.closed_dict['back'] = True
        self.closed_dict['combined'] = True
        self.closed_dict['feelers'] = True
    
    
    
    
    def systemStateCallback(self, msg):
        # ?
        print(msg.state)
        
        self.system_state = msg.state
        if self.file == None and self.system_state == 1:
            self.create_entry()
        elif self.system_state == 3:
            self.close_entry()
            self.close_recording()
        
    
    def deviceStateCallback(self, msg):
        if self.file == None:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_DEVICE, {msg.device}, {msg.state}")
    
    
    def makeTimestamp(self) -> str:
        time = datetime.now()
        frmt = time.strftime("%Y-%m-%d_%H-%M-%S")
        return frmt
    
    def write_file(self, msg):
        if self.file == None:
            return
        
        self.get_logger().info("Writing")
        self.file.write(msg + "\n")
    
    
    def create_entry(self):
        stateFrmt = "autonomous" if self.system_state == 1 else "manual"
        filename = f"{stateFrmt}_{self.makeTimestamp()}"
        
        BASE_PATH = os.path.join(self.home_dir, "Documents", "AutoNav", "Logs", filename)
        os.makedirs(BASE_PATH, exist_ok=True)
        
        self.get_logger().info("Creating Entry")
        self.file = open(os.path.join(BASE_PATH, "log.csv"), "w")
        self.file.write("timestamp, type\n")
    
    def close_entry(self):
        if self.file is None:
            return
        
        self.get_logger().info("Closing Entry")
        self.file.close()
        self.file = None
        
    def imu_feedback(self, msg):
        if not self.config.record_imu:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_IMU, {msg.yaw}, {msg.pitch}, {msg.roll}, {msg.accel_x}, {msg.accel_y}, {msg.accel_z}, {msg.angular_x}, {msg.angular_y}, {msg.angular_z}")
    
    def gps_feedback(self, msg):
        if not self.config.record_gps:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_GPS, {msg.latitude}, {msg.longitude}, {msg.altitude}, {msg.gps_fix}, {msg.is_locked}, {msg.satellites}")
    
    def mfeedback_feedback(self, msg):
        if not self.config.record_motor:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_FEEDBACK, {msg.delta_x}, {msg.delta_y}, {msg.delta_theta}")
        
    def minput_feedback(self, msg):
        if not self.config.record_motor:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_INPUT, {msg.forward_velocity}, {msg.sideways_velocity}, {msg.angluar_velocity}")
    
    def position_feedback(self, msg):
        if not self.config.record_position:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_POSITION, {msg.x}, {msg.y}, {msg.theta}, {msg.latitude}, {msg.longitude}")
        
    def nuc_feedback(self, msg):
        if not self.config.record_nuc:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_PERFORMACE, {msg.timestamp}, {msg.cpu_percentage}, {msg.ram_usage}, {msg.disk_usage}, {msg.gpu_usage}")
    
    def ultrasonic_feedback(self, msg):
        if not self.config.record_ultrasonic:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_ULTRASONIC, {msg.id}, {msg.distance}")
        
    def conbus_feedback(self, msg):
        if not self.config.record_conbus:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_CONBUS, {msg.id}, {msg.data}, {msg.iterator}")
    
    def safetylight_feedback(self, msg):
        if not self.config.record_safetylights:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_SAFETYLIGHT, {msg.autonomous}, {msg.red}, {msg.green}, {msg.blue}")
    
    def performance_feedback(self, msg):
        if not self.config.record_performance:
            return
        
        self.write_file(f"{self.makeTimestamp()}, ENTRY_PERFORMANCE, {msg.name}, {msg.duration}")
    
    
    
        
def main(args=None):
    rclpy.init(args=args)

    playback_sub = playback()

    rclpy.spin(playback_sub)
    playback_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()