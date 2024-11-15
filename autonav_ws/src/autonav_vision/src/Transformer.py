import cv2 as cv
import numpy as np
import rclpy
from types import SimpleNamespace
from cv_bridge import CvBridge
from nav_msgs.msg import MapMetaData, OccupancyGrid
import rclpy.qos
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

import json




g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 100
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0
#### Need to modify #######################################################
class FrameTransformerConfig:

    def __init__(self):
        
        # HSV lower and upper bound value
        self.lower_hue = 0
        self.upper_hue = 180
        self.lower_sat = 0
        self.upper_sat = 255
        self.lower_val = 0
        self.upper_val = 255

        # blur filter
        self.blur_size = 5
        self.blur_iterations = 3
        self.map_res = 80

        # Perspective transform 
        pts1 = np.float32([[80, 200], [400, 220], [480, 640], [0, 640]])
        pts2 = np.float32([[80, 220], [400, 220], [480, 640], [0, 640]])

  
        # Disabling
        self.disable_blur = False
        self.disable_hsv = False
        self.disable_perspective_transform = False

###########################################################################

class FrameTransformer(Node):

    def __init__(self, dir = "left"):
        super().__init__("autonav_vision_transformer")
        self.config = self.get_default_config()
        self.dir = dir

    def init(self):
        self.camera_subscriber = self.create_subscription(CompressedImage, self.directionify("/autonav/camera/compressed") , self.onImageReceived, self.qos_profile)
        self.calibration_subscriber = self.create_subscription(CameraCalibration, "/camera/autonav/calibration", self.onCalibrate)
        self.camera_debug_publisher = self.create_publisher(CompressedImage, self.directionify("/autonav/camera/compressed") + "/cutout", self.qos_profile)
        self.grid_publisher = self.create_publisher(OccupancyGrid, self.directionify("/autonav/cfg_space/raw"), 1)
        self.grid_image_publisher = self.create_publisher(CompressedImage, self.directionify("/autonav/cfg_space/raw/image") + "_small", self.qos_profile)

        self.set_device_state(DeviceStateEnum.OPERATING)

    # ways to do the auto hsv calibration
    # (1) auto hsv color picking
    # (2) Otsu's ?
    def onCalibrate(self, msg: CameraCalibration):

    def config_updated(self, jsonobject):
        self.config = json.loads(self.jdump(jsonobject), object_hook = lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return FrameTransformerConfig

    # set up the Gaussian blur kernel size  
    def get_blur_level(self):
        blur_size = self.config.blur_size
        return (blur_size, blur_size)

    # get the order of points for image transform
    def order_points(self, pts):
        # first create a list of points that form a rectangle
        rect = np.zeros((4,2), dtype = "float32")
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmin(s)]

        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[2] = pts[np.argmin(diff)]

        return rect
    
    # Compute the width and height of the new image/frame
    def compute_max_width_height(self, rect):
        (tl, tr, br, bl) = rect
        widthA = np.linalg.norm(br - bl)
        widthB = np.linalg.norm(tr - tl)
        maxWidth = max(int(widthA), int(widthB))

        heightA = np.linalg.norm(tr - br)
        heightB = np.linalg.norm(tl - bl)
        maxheight = max(int(heightA), int(heightB))

        return maxWidth, maxheight

    # Hough Transform(detect any shape in the image/frame, if the shape can be expressed in a math form)
    def hough_transform(self, pts):
        
        return

        
    # Four point transform for bird view of the frame
    def four_point_transform(self, image, pts):
        if pts.shape != (4, 2):
            raise ValueError("Input Points should be a 4x2 array representing four points.")

        rect = self.order_points(pts)
        maxWidth, maxHeight = self.compute_max_width_height(rect)

        # Define the destination points for the perspective transformation
        dst = np.array([
            [0,0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")
        
        # Compute the perspective transform matrix
        M = cv.getPerspectiveTransform(rect, dst)
        warped = cv.warpPerspective(image, M, (maxWidth, maxHeight))

        return warped

    # Blurred the image to reduced the noise before convert it into hsv colorspaces to prevent external environments interfere
    def blur(self, img):
        if self.config.disable_blur:
            return img
        for _ in range(self.config.blur_iterations):
            img = cv.GaussianBlur(img, self.get_blur_level, 0)

        return img 

    # HSV color space color-picking
    def hsvcolorpicked():
        
        return
    
    # Define the max ROI, should return the bounding box size 
    def roi_max(self, img):
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(img_gray, 127, 255, 0)



        return
    
    # Define the reference ROI, should return the bounding box size
    def roi_reference():
    
        return
    
    # Try to random pick roi within the roi_max
    def random_roi():
        
        return

    # Compare the similarity of colors(with picked hsv color) with detected objects'
    def hsv_calculation():

        return

    # ROI merge operation to get the shape or contour of detected object more accurately
    def roi_merge():

        return
    


    # Convert the Colorspace
    # For the detected object, modify the hsv value until it becomes white
    # return the value to config the upper and lower hsv values
    # Object in the detected image with white color will be desired.
    def get_limits(color):
        c = np.uint8([[color]])
        hsvC = cv.cvtColor(c, cv.COLOR_BGR2HSV)

        lower_limits = hsvC[0][0][0] - 10, 100, 100
        upper_limits = hsvC[0][0][0] + 10, 255, 255

        lower_limits = np.array(lower_limits, dtype = np.uint8)
        upper_limits = np.array(upper_limits, dtype = np.uint8)

        return lower_limits, upper_limits
    # Pixel rejection(2D convolution and filtering, maybe smoothing?)

    # Receive the Image from the camera(front, back, left, right)
    def ImageReceived(self, image: CompressedImage):
        img = g_bridge.compressed_imgmsg_to_cv2(image)
        self.publish_debug_image(img)
        
        # blur it
        img = self.blur(img)
        # Apply filter and return a mask
        
    
def main():
    rclpy.init()
    node_left = ImageTransformer(dir = "left")
    node_right = ImageTransformer(dir = "right")
    Node.run_nodes([node_left, node_right])
    rclpy.shutdown()


if __name__ == "__main__":
    main()



    



    