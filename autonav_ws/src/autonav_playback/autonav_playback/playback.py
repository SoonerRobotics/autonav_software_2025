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
        self.feedback_subscriber = self.create_subscription(MotorFeedback, '/autonav/motorfeedback', self.motor_feedback, self.QOS)
        self.position_subscriber = self.create_subscription(Position, '/autonav/position', self.position_feedback, self.QOS)
        self.nuc_subscriber = self.create_subscription(NUCStatistics, '/autonav/statistics', self.nuc_feedback, self.QOS)
        self.ultrasonics_subscriber = self.create_subscription(Ultrasonic, '/autonav/ultrasonics', self.ultrasonic_feedback, self.QOS)
        self.conbus_subscriber = self.create_subscription(Conbus, '/autonav/CONbus', self.conbus_feedback, self.QOS)
        self.safetylight_subscriber = self.create_subscription(SafetyLights, '/autonav/safety_lights', self.safetylight_feedback, self.QOS)
        self.performance_subscriber = self.create_subscription(Performance, '/autonav/performance', self.performance_feedback, self.QOS)
        
        
        
        # TO-DO
        # Prolly even more camera stuff
        #self.camera_subscriber_left 
        #self.camera_subscriber_right
        #self.astar_subscriber
        
        # self.camera1 = self.create_subscription(CompressedImg, 'autonav/camera/1, self.cameraCallback, self.QOS)
        # self.camera2 = self.create_subscription(CompressedImg, 'autonav/camera/2, self.cameraCallback, self.QOS)
        # self.camera3 = self.create_subscription(CompressedImg, 'autonav/camera/3, self.cameraCallback, self.QOS)
        # self.camera4 = self.create_subscription(CompressedImg, 'autonav/camera/4, self.cameraCallback, self.QOS)
        
        
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
        
        
        # Silly Goofy Code
        self.__topicList = []
        self.__typeList = []
        self.topicDict = {}
        print(len(self.topicDict))
        
        self.topicList_sub = self.create_subscription(String, 'topicList', self.topicListenerCallback, self.QOS)
    
    
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
    
    # Silly Goofy Method
    def topicListenerCallback(self, msg):
        
        msgdata = []
        
        # Parse Topic List
        for s in msg.data.split(' '):
            msgdata.append(s)
        
        # Topics
        for s in msgdata[0].split('/'):
            if s in self.__topicList or s == '':
                continue
            self.__topicList.append(s)
        
        # Types
        for s in msgdata[1].split('/'):
            if s in self.__typeList or s == '':
                continue
            self.__typeList.append(s)
        
        if not len(self.__topicList) == len(self.__typeList):
            return
        
        for i in range(len(self.__topicList)):
            self.topicDict.update({self.__topicList[i]: self.__typeList[i]})
    
    # Serious Stuff below ->
    def systemStateCallback(self, msg):
        # ?
        print(msg.state)
        
        self.system_state = msg.state
        
    
    def deviceStateCallback(self, msg):
        self.write_file(f"{self.makeTimestamp}, ENTRY_DEVICE, {msg.device}, {msg.state}")
    
    
    def makeTimestamp(self) -> str:
        time = datetime.now()
        frmt = time.strftime("%Y-%m-%d_%H-%M-%S")
        return frmt
    
    def write_file(self, msg):
        if self.file == None:
            return
        
        self.file.write(msg + "\n")
    
    
    def create_entry(self):
        stateFrmt = "autonomous" if self.system_state == 0 else "manual"
        filename = f"{stateFrmt}_{self.makeTimestamp()}"
        
        BASE_PATH = os.path.join(self.home_dir, "Documents", "AutoNav", "Logs", filename)
        os.makedirs(BASE_PATH, exist_ok=True)
        
        self.file = open(os.path.join(BASE_PATH, "log.csv"), "w")
        self.file.write("timestamp, type\n")
    
    def close_entry(self):
        if self.file is None:
            return
        
        self.file.close()
        self.file = None
        
    def imu_feedback(self, msg):
        if not self.config.record_imu:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_IMU, {msg.yaw}, {msg.pitch}, {msg.roll}, {msg.accel_x}, {msg.accel_y}, {msg.accel_z}, {msg.angular_x}, {msg.angular_y}, {msg.angular_z}")
    
    def gps_feedback(self, msg):
        if not self.config.record_gps:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_GPS, {msg.latitude}, {msg.longitude}, {msg.altitude}, {msg.gps_fix}, {msg.is_locked}, {msg.satellites}")
    
    def motor_feedback(self, msg):
        if not self.config.record_motor:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_FEEDBACK, {msg.delta_x}, {msg.delta_y}, {msg.delta_theta}")
    
    def position_feedback(self, msg):
        if not self.config.record_position:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_POSITION, {msg.x}, {msg.y}, {msg.theta}, {msg.latitude}, {msg.longitude}")
        
    def nuc_feedback(self, msg):
        if not self.config.record_nuc:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_PERFORMACE", {msg.timestamp}, {msg.cpu_percentage}, {msg.ram_usage}, {msg.disk_usage}, {msg.gpu_usage})
    
    def ultrasonic_feedback(self, msg):
        if not self.config.record_ultrasonic:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_ULTRASONIC, {msg.id}, {msg.distance}")
        
    def conbus_feedback(self, msg):
        if not self.config.record_conbus:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_CONBUS, {msg.id}, {msg.data}, {msg.iterator}")
    
    def safetylight_feedback(self, msg):
        if not self.config.record_safetylights:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_SAFETYLIGHT, {msg.autonomous}, {msg.red}, {msg.green}, {msg.blue}")
    
    def performance_feedback(self, msg):
        if not self.config.record_performance:
            return
        
        self.write_file(f"{self.makeTimestamp}, ENTRY_PERFORMANCE, {msg.name}, {msg.duration}")
    
    
    
        
def main(args=None):
    rclpy.init(args=args)

    playback_sub = playback()

    rclpy.spin(playback_sub)
    playback_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()