import tkinter
from tkinter import filedialog
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from math import sin, cos, tan, atan, pi, radians, degrees

print(f"Starting time: {time.time()}")
startTime = time.time()

# so basically, what we want to do is input a log and get out a map of the course.
# program steps:
# run program
#  => file dialog, select the log? log folder? POSITION.csv and camera.mp3? log.csv and camera.mp3? idk
# run through the csv
# need to split the camera into frames as well
# the things we are looking out for are ENTRY_POSITION (from the particle filter) and ENTRY_CAMERA_IMAGE
#  - except, entry position is relative to the start location of the robot, ENTRY_GPS might be better to acutally plot this on a map
#    so ensure code is flexible enough where making that change is easy (i.e. make it a flag/switch or a boolean variable or something)
# every time we find an ENTRY_POSITION, we look for the closest ENTRY_CAMERA_IMAGE that hasn't yet been used and bind the two together.
# then we pass the camera through a custom transformations.py or something to get a thresholded and flattened image.
#   => but the only thing we really care about are the sides. so just to pixel coordinates (one for left, one for right) that determine robot-relative obstacle bits
# then we have to translate those robot-relative points to global points (taking into account robot position *and* rotation)
# and then we have to plot them or something, along with the robot's position.

# standard tkinter boilerplate, https://github.com/Team-OKC-Robotics/FRC-2023/blob/master/process_wpilib_logs.py
root = tkinter.Tk()
root.withdraw()

# get the filename of the CSV log
log_filename = filedialog.askopenfilename()

# read all the lines in the CSV into one big array
#TODO add support for multiple runs or something
log = []
if log_filename != None and log_filename != "":
    with open(log_filename, "r") as csv:
        log = csv.readlines()

# get the filename of the camera
video_filename = filedialog.askopenfilename()

print(f"Length of log csv, line 42: {len(log)}")

# and make an array of all the individual frames in the video
frames = []
video = cv2.VideoCapture(video_filename)
while video.isOpened():
    ret, image = video.read()

    if not ret:
        break # we've reached the end of the video

    frames.append(image)

print(f"Length of frames, line 49: {len(frames)}")

# the code below verifies that there is a ENTRY_POSITION between every ENTRY_CAMERA_IMAGE, essentially meaning that whenever we hit a camera image,
# we are clear to use the last position entry, because it won't collide with another camera's position entry.
# cameraIndex = 0
# lastCameraIndex = 0
# posIndex = 0
# hitCount = 0
# for idx, line in enumerate(log):
#     # entry = line.split(",")
#     entry = [item.strip() for item in line.split(",")]

#     if entry[1] == "ENTRY_CAMERA_IMAGE":
#         cameraIndex = idx
#     elif entry[1] == "ENTRY_POSITION":
#         posIndex = idx
    
#     # print(lastCameraIndex, posIndex, cameraIndex)

#     # there should always be a position index between camera frames
#     if not (lastCameraIndex < posIndex < cameraIndex) and not cameraIndex == lastCameraIndex:
#         print(f"FOUND ONE! at line: {idx}")
#         hitCount += 1
    
#     if hitCount > 10:
#         print(lastCameraIndex)
#         raise SystemExit
    
#     lastCameraIndex = cameraIndex

# print("FINISHING")
# raise SystemExit


#TODO FIXME
class Point:
    def __init__(self, x, y, heading, lat, lon, image):
        self.x = x
        self.y = y
        self.heading = heading
        self.lat = lat
        self.lon = lon
        self.image = image

        self.leftX = 0
        self.rightX = 0

# go through the log array and video array to find the entries that correspond to each other
positionIndex = 0
frameIndex = 0
lastFrameIndex = None
frameCount = -1
combinedData = [] # a list of POSITIONs (or GPS coordinates) and corresponding CAMERA_IMAGEs

for idx, line in enumerate(log):
    entry = line.split(",") # it's a csv, so split along commas to get data

    match entry[1].strip():
        case "ENTRY_POSITION":
            positionIndex = idx
        
        case "ENTRY_CAMERA_IMAGE":
            frameIndex = idx # index in the massive CSV of the image
            frameCount += 1 # index in the video
        
        case _:
            continue # ignore anything else
    
    # if the frame index has changed (i.e. we've encountered a new frame)
    # (and this isn't the first frame we've encountered, but that only matters for at the start)
    if frameIndex != lastFrameIndex and lastFrameIndex is not None:
        # see https://github.com/SoonerRobotics/autonav_software_2024/blob/main/autonav_ws/src/autonav_playback/src/playback.py line 187
        x, y, heading, lat, lon = [float(item.strip()) for item in log[positionIndex].split(",")[2:]] # some horrendous python

        image = frames[frameCount]

        # print("AAAAAAAAAAAAAAAAAAAAAA")

        #TODO do something with the timestamp? interpolate points based on velocity? splines? should probably do something like that.

        combinedData.append(Point(x, y, heading, lat, lon, image)) # then append the point

    
    lastFrameIndex = frameIndex

print(f"combined data at lin 140: {len(combinedData)}")

# image processing taken from https://github.com/SoonerRobotics/autonav_software_2024/blob/f8f0925f5d4d29e61f98bea63b09dc5598481e54/autonav_ws/src/autonav_vision/src/feeler.py
# which was itself taken from last year's (2024) vision pipeline in transformations.py, but this version is made to work with the combined image instead of individual images, so it'
# more suited to our needs, and also already done

# image shape is 800x1600x3; 1600 because it's two 800x800 side by side because dual camera
WIDTH = 960
HEIGHT = 640

# verticies for region-of-disinterest
# order is top-left, top-right, bottom-right, bottom-left
VERTICIES = (
    (285, 450),
    (616, 450),
    (722, 639),
    (262, 639)
)

# HSV thresholding values for obstacle detection
lower = (0, 0, 0)
upper = (255, 95, 210)

# kernel for erode/dilate
kernel = cv2.getStructuringElement(2, (2, 2))

def threshold(image):
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, lower, upper)
    mask = 255 - mask

    mask = cv2.fillConvexPoly(mask, np.array(VERTICIES, dtype=np.int32), (0))

    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    
    return mask

def centerCoordinates(x, y):
    return (x + WIDTH//2), (y + HEIGHT//2)

# now we need to go through the frames and process them.
# essentially, we're looking for the points directly to the left and right of the robot where there's an obstacle
# we don't want to look forwards, because danger squiggle camera, and no good way to flatten, and so on.
Y = -150 #TODO FIXME

for point in combinedData:
    point.image = threshold(point.image)

    # check left for obstacles
    for x in range(0, -WIDTH//2 + 2, -1): # for every pixel along y, going to the left from the center of the image
        check_x, check_y = centerCoordinates(x, y)

        # print(check_y, check_x)

        # if any of the pixel's color values (in RGB I think) are > 0 then
        if point.image[int(check_y), int(check_x)].any() > 0:
            # that is an obstacle, and we have found our point
            point.leftX = x
            break
    else:
        # point.leftX = None #FIXME?
        point.leftX = 0
    
    # repeat, check right for obstacles FIXME I don't like this duplicated code right here. is annoying when have to debug and fix
    for x in range(0, WIDTH//2 - 2, 1): # for every pixel along y, going to the right from the center of the image
        check_x, check_y = centerCoordinates(x, y)

        # if any of the pixel's color values (in RGB I think) are > 0 then
        if point.image[int(check_y), int(check_x)].any() > 0:
            # that is an obstacle, and we have found our point
            point.rightX = x
            break
    else:
        # point.rightX = None #FIXME?
        point.rightX = 0



# now that we have all the data we need, we just need to plot it

# at the start, from the robot to the right lane, is approximately the width of the right camera image, which is 480 pixels.
# those lanes should be about 10 feet apart, according to the rules, I think, and the right half of the robot in that frame is 1 foot.
# so that's (5-1) = 4 feet to 480 pixels, which is 1.219 meters per 480 pixels, right up at the edge of the front of the robot, approximately
pixelsToMeters = 1.219 / 480

# turn list of point objects into a list of x and y coordinates
x_coords = []
y_coords = []
robot_x_coords = []
robot_y_coords = []
for point in combinedData:
    robot_x_coords.append(point.x)
    robot_y_coords.append(point.y)

    #FIXME this skips the points without obstacle data in them, fix so it just displays these in a different color or something
    # maybe append to a different list and scatter that in a different color?
    if point.leftX == point.rightX == 0:
        continue

    #FIXME this doesn't take into account the robot's rotation/heading (just need to do like point.x + point.leftX * cos(theta)) or something
    x_coords.append(point.x - (point.leftX * pixelsToMeters * sin(point.heading)))
    y_coords.append(point.y - (point.leftX * pixelsToMeters * cos(point.heading)))

    x_coords.append(point.x + (point.rightX * pixelsToMeters * sin(point.heading)))
    y_coords.append(point.y + (point.rightX * pixelsToMeters * cos(point.heading)))

    #TODO plot the actual robot's position, preferably in a different color


print(len(combinedData))
print(len(x_coords))
print(len(y_coords))

endingTime = time.time()
print(f"Ending time: {endingTime}")
print(f"Time take: {endingTime - startTime}s")

#TODO FIXME these should be all in GPS coordinates units instead of relative x/y units so we can have consistency and actually map the course
plt.figure()
# plt.plot(x_coords, y_coords) # speed (based on timestamp, or like, a log message or something) could be color? not useful in the slightest htough, this is a stationairy map. nevermind.
plt.scatter(x_coords, y_coords, 2, color="#BB0000") #TODO points without obstacles associated with them (i.e. if leftX and rightX are both 0) should be a different color or just not plotted or something
plt.scatter(robot_x_coords, robot_y_coords, 1, color="#0000BB")
plt.show()