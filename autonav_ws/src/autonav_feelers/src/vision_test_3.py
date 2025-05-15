import cv2
import numpy as np

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class Config:
    def __init__(self):
        self.topLeftX = 0
        self.topLeftY = 0
        self.topRightX = IMAGE_WIDTH
        self.topRightY = 0
        self.bottomLeftX = 0
        self.bottomLeftY = IMAGE_HEIGHT
        self.bottomRightX = IMAGE_WIDTH
        self.bottomRightY = IMAGE_HEIGHT
    
src_config = Config()
dest_config = Config()

def read_image():
    image = cv2.imread("./data/unflattened_front.png")
    return image

def do_flatten(image):
    src_pts = np.float32([(src_config.topLeftX, src_config.topLeftY), (src_config.topRightX, src_config.topRightY), (src_config.bottomLeftX, src_config.bottomLeftY), (src_config.bottomRightX, src_config.bottomRightY)])
    dest_pts = np.float32([(dest_config.topLeftX, dest_config.topLeftY), (dest_config.topRightX, dest_config.topRightY), (dest_config.bottomLeftX, dest_config.bottomLeftY), (dest_config.bottomRightX, dest_config.bottomRightY)])
    matrix = cv2.getPerspectiveTransform(src_pts, dest_pts)
    image = cv2.warpPerspective(image, matrix, (640, 480))

    cv2.imshow("flattened", image)
    keycode = cv2.waitKey(10)

    return keycode

def onSrcTopLeftXChange(val):
    src_config.topLeftX = val

def onSrcTopLeftYChange(val):
    src_config.topLeftY = val

def onSrcTopRightXChange(val):
    src_config.topRightX = val

def onSrcTopRightYChange(val):
    src_config.topRightY = val

def onSrcBottomLeftXChange(val):
    src_config.bottomLeftX = val

def onSrcBottomLeftYChange(val):
    src_config.bottomLeftY = val

def onSrcBottomRightXChange(val):
    src_config.bottomRightX = val

def onSrcBottomRightYChange(val):
    src_config.bottomRightY = val

def onDestTopLeftXChange(val):
    dest_config.topLeftX = val

def onDestTopLeftYChange(val):
    dest_config.topLeftY = val

def onDestTopRightXChange(val):
    dest_config.topRightX = val

def onDestTopRightYChange(val):
    dest_config.topRightY = val

def onDestBottomLeftXChange(val):
    dest_config.bottomLeftX = val

def onDestBottomLeftYChange(val):
    dest_config.bottomLeftY = val

def onDestBottomRightXChange(val):
    dest_config.bottomRightX = val

def onDestBottomRightYChange(val):
    dest_config.bottomRightY = val

# make trackbars
cv2.namedWindow('source points')
cv2.createTrackbar('TL X', "source points", 0, 640, onSrcTopLeftXChange)
cv2.createTrackbar('TL - Y', "source points", 0, 480, onSrcTopLeftYChange)
cv2.createTrackbar('TR X', "source points", 0, 640, onSrcTopRightXChange)
cv2.createTrackbar('TR - Y', "source points", 0, 480, onSrcTopRightYChange)
cv2.createTrackbar('BL X', "source points", 0, 640, onSrcBottomLeftXChange)
cv2.createTrackbar('BL - Y', "source points", 0, 480, onSrcBottomLeftYChange)
cv2.createTrackbar('BR X', "source points", 0, 640, onSrcBottomRightXChange)
cv2.createTrackbar('BR - Y', "source points", 0, 480, onSrcBottomRightYChange)

cv2.namedWindow('dest points')
cv2.createTrackbar('TL X', "dest points", 0, 640, onDestTopLeftXChange)
cv2.createTrackbar('TL - Y', "dest points", 0, 480, onDestTopLeftYChange)
cv2.createTrackbar('TR X', "dest points", 0, 640, onDestTopRightXChange)
cv2.createTrackbar('TR - Y', "dest points", 0, 480, onDestTopRightYChange)
cv2.createTrackbar('BL X', "dest points", 0, 640, onDestBottomLeftXChange)
cv2.createTrackbar('BL - Y', "dest points", 0, 480, onDestBottomLeftYChange)
cv2.createTrackbar('BR X', "dest points", 0, 640, onDestBottomRightXChange)
cv2.createTrackbar('BR - Y', "dest points", 0, 480, onDestBottomRightYChange)


# topLeftX
# topLeftY
# topRightX
# topRightY
# bottomLeftX
# bottomLeftY
# bottomRightX
# bottomRightY

while True:
    if do_flatten(read_image()) == ord("q"):
        cv2.destroyAllWindows()
        raise SystemExit