import rclpy
from rclpy.node import Node

from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import *
from cv_bridge import CvBridge

from datetime import datetime
import os

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

        
        
        
        # TO-DO
        # Raw Camera Record Settings
        self.record_command = ['ffmpeg','-y', '-loglevel', 'error', '-f', 'image2pipe', '-vcodec', 'mjpeg', '-framerate', '15', '-s', '640x480', '-i', '-']
        # Vision Pipeline record Settings
        self.vis_command = ['ffmpeg','-y', '-loglevel', 'error', '-f', 'image2pipe', '-vcodec', 'mjpeg', '-framerate', '15', '-s', '640x480', '-i', '-', 'output.mp4']
        
        # Raw Camera Topics
        self.camera1 = self.create_subscription(CompressedImage, 'autonav/camera/left', lambda msg: self.cameraCallback(msg, 'left'), self.QOS)
        self.camera2 = self.create_subscription(CompressedImage, 'autonav/camera/right', lambda msg: self.cameraCallback(msg, 'right'), self.QOS)
        self.camera3 = self.create_subscription(CompressedImage, 'autonav/camera/front', lambda msg: self.cameraCallback(msg, 'front'), self.QOS)
        self.camera4 = self.create_subscription(CompressedImage, 'autonav/camera/back', lambda msg: self.cameraCallback(msg, 'back'), self.QOS)
        
        # FFmpeg Process Dict
        self.process_dict = {'left':None, 'right':None, 'front':None, 'back':None}
        self.closed_dict = {'left':False, 'right':False, 'front':False, 'back':False}
        
        
        
        # FFmpeg record video
        # ffmpeg -f avfoundation -framerate 30 -i "0:1" output.mp4
        # Might need to change avfoundation, framerate and -i
        # Use Pipe? Might need to change codec?
        # ffmpeg -f image2pipe -vcodec mjpec -framerate 30 -i - output.mp4
        # command = ['ffmpeg','-y', '-loglevel', 'error', '-f', 'image2pipe', '-vcodec', 'mjpeg', '-framerate', '30', '-s', '1920x1080', '-i', '-', 'output.mp4']
        
        # Dev Video Subscriber
        self.dev_vid_sub = self.create_subscription(CompressedImage, 'dev_vid', self.dev_vid_feedback, self.QOS)
        self.dev_vid_status_sub = self.create_subscription(Bool, 'vid_status', self.close_proc, self.QOS)
        self.process = None
        self.closed = False
        
    
    def cameraCallback(self, msg, id):
        
        if id == 'left' and self.process_dict["left"] == None and not self.closed_dict['left']:
            self.get_logger().info('Opening New Process')
            
            # Adjust the command output filename
            c = self.record_command
            c.append('left.mp4')
            self.process_dict["left"] = subprocess.Popen(c, stdin=subprocess.PIPE)
            
            self.process_img(msg, self.process_dict["left"])
        elif id == 'right' and self.process_dict["right"] == None and not self.closed_dict['right']:
            self.get_logger().info('Opening New Process')
            
            # Adjust the command output filename
            c = self.record_command
            c.append('right.mp4')
            self.process_dict["right"] = subprocess.Popen(c, stdin=subprocess.PIPE)
            
            self.process_img(msg, self.process_dict["right"])
        elif id == "front" and self.process_dict["front"] == None and not self.closed_dict['front']:
            self.get_logger().info('Opening New Process')
            
            # Adjust the command output filename
            c = self.record_command
            c.append('front.mp4')
            self.process_dict["front"] = subprocess.Popen(c, stdin=subprocess.PIPE)
            
            self.process_img(msg, self.process_dict["front"])  
        elif id == "back" and self.process_dict["back"] == None and not self.closed_dict['back']:
            self.get_logger().info('Opening New Process')
            
            # Adjust the command output filename
            c = self.record_command
            c.append('back.mp4')
            self.process_dict["back"] = subprocess.Popen(c, stdin=subprocess.PIPE)
            
            self.process_img(msg, self.process_dict["back"])
        
        if id == 'left' and not self.closed_dict['left']:
            self.process_img(msg, self.process_dict["left"])
        elif id == 'right' and not self.closed_dict['right']:
            self.process_img(msg, self.process_dict["right"])
        elif id == 'front' and not self.closed_dict['front']:
            self.process_img(msg, self.process_dict["front"])
        elif id == 'back' and not self.closed_dict['back']:
            self.process_img(msg, self.process_dict["back"])
    
    def process_img(self, msg, process):
        
        image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        success, encoded_image = cv2.imencode('.jpg', image)
            
        if success:
            process.stdin.write(encoded_image.tobytes())
            self.get_logger().info('Writing Frame')
        else:
            print("Image Encoding Failed")
    
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
        
        self.process_dict['left'] = None
        self.process_dict['right'] = None
        self.process_dict['front'] = None
        self.process_dict['back'] = None
        
        self.closed_dict['left'] = True
        self.closed_dict['right'] = True
        self.closed_dict['front'] = True
        self.closed_dict['back'] = True
    
    
    def dev_vid_feedback(self, msg):
        if self.process == None and self.closed == False:
            
            self.get_logger().info('Opening New Process')
            command = ['ffmpeg','-y', '-loglevel', 'error', '-f', 'image2pipe', '-vcodec', 'mjpeg', '-framerate', '8', '-s', '1280x720', '-i', '-', 'output.mp4']
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE)
            
            image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            success, encoded_image = cv2.imencode('.jpg', image)
            
            if success:
                self.process.stdin.write(encoded_image.tobytes())
                self.get_logger().info('Writing Frame')
            else:
                print("Image Encoding Failed")        
        elif self.closed == False:
            image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            success, encoded_image = cv2.imencode('.jpg', image)
            if success:
                self.process.stdin.write(encoded_image.tobytes())
                self.get_logger().info('Writing Frame')
            else:
                print("Image Encoding Failed")
    
    def close_proc(self, msg):
        if not self.process == None:
            self.get_logger().info('Closing Process')
            self.process.stdin.close()
            self.process.wait()
            self.process = None
            self.closed = True
    
    
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