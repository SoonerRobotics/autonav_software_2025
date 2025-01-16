import rclpy
from rclpy.node import Node

from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import *
from cv_bridge import CvBridge

import cv2

import PIL
from PIL import Image
import numpy as np

bridge = CvBridge()

# refrence for combining images https://stackoverflow.com/questions/30227466/combine-several-images-horizontally-with-python

class combinedView(Node):
    
    def __init__(self):
        super().__init__("combined")
        self.QOS = 10
        
        self.frame_num = [0,0,0,0]
        
        self.left = []
        self.right = []
        self.front = []
        self.back = []
        
        # Subscribe to Camera Topics
        self.camera1 = self.create_subscription(CompressedImage, 'autonav/camera/left', lambda msg: self.camera_callback(msg, 'left'), self.QOS)
        self.camera2 = self.create_subscription(CompressedImage, 'autonav/camera/right', lambda msg: self.camera_callback(msg, 'right'), self.QOS)
        self.camera3 = self.create_subscription(CompressedImage, 'autonav/camera/front', lambda msg: self.camera_callback(msg, 'front'), self.QOS)
        self.camera4 = self.create_subscription(CompressedImage, 'autonav/camera/back', lambda msg: self.camera_callback(msg, 'back'), self.QOS)
        
        # Combined View Publisher
        self.combined_pub = self.create_publisher(CompressedImage, 'combined', self.QOS)
        
    def camera_callback(self, msg, id):
        
        if id == 'left':
            self.left.append(bridge.compressed_imgmsg_to_cv2(msg, "bgr8"))
            self.frame_num[0] = len(self.left)
        elif id == 'right':
            self.right.append(bridge.compressed_imgmsg_to_cv2(msg, "bgr8"))
            self.frame_num[1] = len(self.right)
        elif id == 'front':
            self.front.append(bridge.compressed_imgmsg_to_cv2(msg, "bgr8"))
            self.frame_num[2] = len(self.front)
        elif id == 'back':
            self.back.append(bridge.compressed_imgmsg_to_cv2(msg, "bgr8"))
            self.frame_num[3] = len(self.back)
        
        if self.compare_frame_counter():
            print(self.frame_num, self.left)
            self.combine(self.left[0], self.right[0], self.front[0], self.back[0])
    
    # Images in cv2 format
    def combine(self, left, right, front, back):
        image_cv = [left,right,front,back]
        image_rgb = []
        image_pil = []
        for i in image_cv:
            image_rgb.append(cv2.cvtColor(i, cv2.COLOR_BGR2RGB))
        for i in image_rgb:
            image_pil.append(Image.fromarray(i))
        # Combine Left and right
        #open_pil = [Image.open(image_pil[0]), Image.open(image_pil[1])]
        top = np.hstack([image_pil[0], image_pil[1]])
        bottom = np.hstack([image_pil[2], image_pil[3]])
        final = np.vstack([top, bottom])
        final = cv2.cvtColor(final, cv2.COLOR_RGB2BGR)
        msg = bridge.cv2_to_compressed_imgmsg(final)
        
        # Delete Used Frames
        del self.left[0]
        del self.right[0]
        del self.front[0]
        del self.back[0]
        self.frame_num = [len(self.left), len(self.right), len(self.front), len(self.back)]
        
        # Publish Message
        self.combined_pub.publish(msg)
    
    def compare_frame_counter(self):
        if 0 in self.frame_num:
            return False
        return True
        
def main(args=None):
    rclpy.init(args=args)
    combined_view = combinedView()
    rclpy.spin(combined_view)
    combined_view.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()