import rclpy
from rclpy.node import Node

from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import *
from cv_bridge import CvBridge


from datetime import datetime
import os
import cv2

import subprocess

bridge = CvBridge()


class TestPublisher(Node):

    def __init__(self):
        super().__init__('dev_vid')
        
        # Testing Publishers
        self.pub = []
        self.pub.append(self.create_publisher(SystemState, 'autonav/shared/system', 10))
        self.pub.append(self.create_publisher(DeviceState, 'autonav/shared/device', 10))
        
        self.pub.append(self.create_publisher(IMUData, '/autonav/imu', 10))
        self.pub.append(self.create_publisher(GPSFeedback, '/autonav/gps', 10))
        self.pub.append(self.create_publisher(MotorFeedback, '/autonav/motorfeedback', 10))
        self.pub.append(self.create_publisher(Position, '/autonav/position', 10))
        self.pub.append(self.create_publisher(NUCStatistics, '/autonav/statistics', 10))
        self.pub.append(self.create_publisher(Ultrasonic, '/autonav/ultrasonics', 10))
        self.pub.append(self.create_publisher(Conbus, '/autonav/CONbus', 10))
        self.pub.append(self.create_publisher(SafetyLights, '/autonav/safety_lights', 10))
        self.pub.append(self.create_publisher(Performance, '/autonav/performance', 10))
        
        
        timer_period =  0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        
        

    def timer_callback(self):
        
        if self.i >= 50:
            msg = SystemState()
            msg.state = 3
            self.pub[0].publish(msg)
            return
        
        self.get_logger().info('Preparing Test Publish')
        # Cancer Ahead!!!
        for p in self.pub:
            
            if p.msg_type == SystemState:
                msg = SystemState()
                msg.state = 1
                p.publish(msg)
                continue
            elif p.msg_type == DeviceState:
                pass
            elif p.msg_type == IMUData:
                msg = IMUData()
                msg.yaw = 0.0
                msg.pitch = 0.0
                msg.roll = 0.0
                msg.accel_x = 0.0
                msg.accel_y = 0.0
                msg.accel_z = 0.0
                msg.angular_x = 0.0
                msg.angular_y = 0.0
                msg.angular_z = 0.0
                p.publish(msg)
                continue
            elif p.msg_type == GPSFeedback:
                msg = GPSFeedback()
                msg.latitude = 0.0
                msg.longitude = 0.0
                msg.altitude = 0.0
                msg.gps_fix = 0
                msg.is_locked = False
                msg.satellites = 0
                p.publish(msg)
                continue
            elif p.msg_type == MotorFeedback:
                msg = MotorFeedback()
                msg.delta_x = 0.0
                msg.delta_y = 0.0
                msg.delta_theta = 0.0
                p.publish(msg)
                continue
            elif p.msg_type == Position:
                msg = Position()
                msg.x = 0.0
                msg.y = 0.0
                msg.theta = 0.0
                msg.latitude = 0.0
                msg.longitude = 0.0
                p.publish(msg)
                continue
            elif p.msg_type == NUCStatistics:
                msg = NUCStatistics()
                msg.timestamp = 0
                msg.cpu_percentage = 0.0
                msg.ram_usage = 0.0
                msg.disk_usage = 0.0
                msg.gpu_usage = 0.0
                p.publish(msg)
                continue
            elif p.msg_type == Ultrasonic:
                msg = Ultrasonic()
                msg.id = 0
                msg.distance = 0
                p.publish(msg)
                continue
            elif p.msg_type == Conbus:
                msg = Conbus()
                msg.id = 0
                msg.data = [0]
                msg.iterator = -1
                p.publish(msg)
                continue
            elif p.msg_type == SafetyLights:
                msg = SafetyLights()
                msg.autonomous = False
                msg.red = 0
                msg.green = 0
                msg.blue = 0
                p.publish(msg)
                continue
            elif p.msg_type == Performance:
                msg = Performance()
                msg.name = "TEST"
                msg.duration = 0
                p.publish(msg)
                continue
        
        self.get_logger().info(f"Publishing. i: {self.i}")
        self.i += 1
        return
        
        


def main(args=None):
    rclpy.init(args=args)

    tpub = TestPublisher()

    rclpy.spin(tpub)
    tpub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()