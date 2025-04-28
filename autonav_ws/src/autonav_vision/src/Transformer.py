from types import SimpleNamespace
import rclpy
import cv2 as cv
import numpy as np

from nav_msgs.msg import MapMetaData, OccupancyGrid
import rclpy.qos
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from scr.states import SystemModeEnum
import json

from scr.node import Node
from scr.states import DeviceStateEnum
from scr.utils import clamp

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
        self.left_topleft = [80, 220]
        self.left_topright = [400, 220]
        self.left_bottomright = [480, 640]
        self.left_bottomleft = [0, 640]
        self.right_topleft = [80, 220]
        self.right_topright = [400, 220]
        self.right_bottomright = [480, 640]
        self.right_bottomleft = [0, 640]

        # Hough Transform
        self.edge_thresholdOne = 50
        self.edge_thresholdTwo = 150
        self.theta_resolution = 1
        self.rho_resolution = 1
        self.line_threshold = 100
        self.min_line_length = 50
        self.max_line_gap = 10


        self.top_width = 320
        self.bottom_width = 240
        self.offset = 20
        
        # ROI picker
        self.reduction_percentage = 10
        
        # Disabling method
        self.disable_blur = False
        self.disable_hsv = False
        self.disable_perspective_transform = False

###########################################################################

class FrameTransformer(Node):

    def __init__(self, dir = "left"):
        super().__init__("autonav_vision_transformer")
        self.config = self.get_default_config()
        self.dir = dir

    def directionify(self, topic):
        return topic + "/" + self.dir
    
    def init(self):
        self.camera_subscriber = self.create_subscription(CompressedImage, self.directionify("/autonav/camera/compressed") , self.onImageReceived, self.qos_profile)
        self.calibration_subscriber = self.create_subscription(CameraCalibration, "/camera/autonav/calibration", self.onCalibrate)
        self.camera_debug_publisher = self.create_publisher(CompressedImage, self.directionify("/autonav/camera/compressed") + "/cutout", self.qos_profile)
        self.grid_publisher = self.create_publisher(OccupancyGrid, self.directionify("/autonav/cfg_space/raw"), 1)
        self.grid_image_publisher = self.create_publisher(CompressedImage, self.directionify("/autonav/cfg_space/raw/image") + "_small", self.qos_profile)
        self.set_device_state(DeviceStateEnum.OPERATING)

    def config_updated(self, jsonobject):
        self.config = json.loads(self.jdump(jsonobject), object_hook = lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return FrameTransformerConfig

    def hough_transform(self, image):
        """Hough Transform(detect any shape in the image/frame, if the shape can be expressed in a math form)"""
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        edges = cv.Canny(gray, self.config.edge_thresholdOne, self.config.edge_thresholdTwo)
        lines = cv.HoughLines(edges, self.config.rho_resolution, np.deg2rad(self.config.theta_resolution), self.config.line_threshold)
        return lines

    def order_points(self, pts):
        """
        Get the order of points for image transform. Use a convex hull approach combined
        with geometric sorting to avoid the failure of drawing four corner points for skewed shapes
        """
        # first create a list of points that form a rectangle
        rect = np.zeros((4,2), dtype = "float32")
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmin(s)]

        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[2] = pts[np.argmin(diff)]

        return rect
    
    def compute_max_width_height(self, rect):
        """Compute the width and height of the new image/frame"""
        (tl, tr, br, bl) = rect
        widthA = np.linalg.norm(br - bl)
        widthB = np.linalg.norm(tr - tl)
        maxWidth = max(int(widthA), int(widthB))

        heightA = np.linalg.norm(tr - br)
        heightB = np.linalg.norm(tl - bl)
        maxheight = max(int(heightA), int(heightB))

        return maxWidth, maxheight

    def four_point_transform(self, image, pts):
        """Four point transform for bird eye view of the frame"""
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

    def get_blur_level(self):
        """Set up the Gaussian blur kernel size"""
        blur_size = self.config.blur_size
        return (blur_size, blur_size)
    
    def blur(self, img):
        """Blurred the image to reduced the noise before convert it into hsv colorspaces to prevent external environment interfere"""
        if self.config.disable_blur:
            return img
        for _ in range(self.config.blur_iterations):
            img = cv.GaussianBlur(img, self.get_blur_level, 0)

        return img 
    
    def apply_perspective_tranform(self, img):
        #TODO
        
    def publish_debug_image(self, img):
        """Publish the modified image with roi, hough Transform and Perspective Transform"""
        img_copy = img.copy()

        # Draw region of interest (parallelogram)
        roi_points = np.array([
            [self.config.roi_x1, self.config.roi_y1],
            [self.config.roi_x2, self.config.roi_y2], 
            [self.config.roi_x3, self.config.roi_y3],
            [self.config.roi_x4, self.config.roi_y4]
        ], np.int32)
        cv.polylines(img_copy, [roi_points], True, (255, 0, 0), 2)  # Green color for ROI

        # Perform Hough Transform and draw results
        lines = self.hough_transform(img)
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                cv.line(img_copy, (x1,y1), (x2,y2), (0,255,0), 2)
        
        # Draw perspective transform points
        if self.dir == "left":
            pts = [self.config.left_topleft, self.config.left_topright, self.config.left_bottomright, self.config.left_bottomleft]
        else:
            [self.config.right_topleft, self.config.right_topright, self.config.right_bottomright, self.config.right_bottomleft]
        cv.polylines(img_copy, [np.array(pts)], True, (0, 0, 255), 2)

        self.camera_debug_publisher.publish(g_bridge.cv_to_compressed_imgmsg(img_copy))

    

    # colorTracker class
    class ColorTracker:

        def __init__(self):
            self.roi = None
            self.lower = np.array([0, 0, 0])
            self.upper = np.array([179, 255, 255])
            self.alpha = 0.1
            self.min_area = 500
            self.cap = None
            self.roi_start = None
            self.roi_end = None
            self.selecting = False

        def initialize_camera(self):
            self.cap = cv.VideoCapture(0)
            return self.cap.isOpened()
        
        def mouse_callback(self, event, x, y, flags, param):
            if event == cv.EVENT_LBUTTONDOWN:
                self.roi_start = (x, y)
                self.selecting = True
            elif event == cv.EVENT_MOUSEMOVE and self.selecting:
                self.roi_end = (x, y)
            elif event == cv.EVENT_LBUTTONUP:
                self.roi_end = (x, y)
                self.selecting = False
        
        def select_roi(self, frame):
            """Select region of interest"""
            cv.namedWindow('Select ROI')
            cv.setMouseCallback('Select ROI', self.mouse_callback)
        
            while True:
                display = frame.copy()
                if self.roi_start and self.roi_end:
                    cv.rectangle(display, self.roi_start, self.roi_end, (0, 255, 0), 2)
                cv.imshow('Select ROI', display)
            
                key = cv.waitKey(1) & 0xFF
                if key == 13:  # Enter key
                    break
                elif key == 27:  # ESC key
                    return False
        
            cv.destroyWindow('Select ROI')
            self.roi = (self.roi_start[0], self.roi_start[1],
                        self.roi_end[0] - self.roi_start[0],
                        self.roi_end[1] - self.roi_start[1])
            return True
        
        def initialCalibrate(self, frame):
            """perform initial calibration"""
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            x, y, w, h = self.roi
            roi_region = hsv[y:y+h, x:x+w]

            h_mean, h_std = np.mean(roi_region[:,:,0]), np.std(roi_region[:,:,0])
            s_mean, s_std = np.mean(roi_region[:,:,1]), np.std(roi_region[:,:,1])
            v_mean, v_std = np.mean(roi_region[:,:,2]), np.std(roi_region[:,:,2])

            self.lower = np.array([
            max(0, h_mean - 2*h_std),
            max(0, s_mean - 2*s_std),
            max(0, v_mean - 2*v_std)
            ])
            self.upper = np.array([
                min(179, h_mean + 2*h_std),
                min(255, s_mean + 2*s_std),
                min(255, v_mean + 2*v_std)
            ])
        
        
        def process_frame(self, frame):
            """Process a single frame and return results"""
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, self.lower, self.upper)
            
            # Noise Reduction
            kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

            # Stray pixel rejection
            cleaned_mask = np.zeros_like(mask)
            num_labels, labels, stats, _ = cv.connectedComponentsWithStats(mask, 8, cv.CV_32S)
            
            for i in range(1, num_labels):
                if stats[i, cv.CC_STAT_AREA] >= self.min_area:
                    cleaned_mask[labels == i] = 255

            # Find largest Contour
            contours, _ = cv.findContours(cleaned_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key = cv.contourArea) if contours else None

            return cleaned_mask, largest_contour
        
        def update_thresholds(self, detected_pixels):
            """Adaptively adjust thresholds"""
            if len(detected_pixels) < 100:
                return
            
            # Update hsv value
            new_h = np.mean(detected_pixels[:,0])
            new_s = np.mean(detected_pixels[:,1])
            new_v = np.mean(detected_pixels[:,2])
            
            self.lower = np.clip([
                self.alpha*new_h + (1-self.alpha)*self.lower[0],
                self.alpha*new_s + (1-self.alpha)*self.lower[1],
                self.alpha*new_v + (1-self.alpha)*self.lower[2]
                ])
            
            self.upper = np.clip([
                self.alpha*new_h + (1-self.alpha)*self.upper[0],
                self.alpha*new_s + (1-self.alpha)*self.upper[1],
                self.alpha*new_v + (1-self.alpha)*self.upper[2]
                ])
        
        def get_target_position(self, contour):
            """Get centroid of largest detected object"""
            if contour is None:
                return None
            M = cv.moments(contour)
            if M["m00"] == 0:
                return None
            return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
        
        def release(self):
            """Release Measures"""
            if self.cap:
                self.cap.release()
            cv.destroyAllWindows()


    def onImageReceived(self, image: CompressedImage):
        # Decompress image
        img = g_bridge.compressed_imgmsg_to_cv2(image)

        # Publish debug image
        self.publish_debug_image(img)

        # Blur it up
        img = self.blur(img)

        # Apply filter and return a mask
        img = self.apply_hsv(img)
        
        # Get the ROI
        img = self.ColorTracker.select_roi(img)

        # Apply perspective transform
        img = self.apply_perspective_transform(img)

        # Actually generate the map
        self.publish_occupancy_grid(img)
        
    

def main():
    rclpy.init()
    node_left = ImageTransformer(dir = "left")
    node_right = ImageTransformer(dir = "right")
    Node.run_nodes([node_left, node_right])
    rclpy.shutdown()


if __name__ == "__main__":
    main()



    



    