import tkinter
from tkinter import filedialog
import cv2
import numpy as np

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
log = []
if log_filename != None and log_filename != "":
    with open(log_filename, "r") as csv:
        log = csv.readlines()

# get the filename of the camera
video_filename = filedialog.askopenfilename()

# and make an array of all the individual frames in the video
frames = []
video = cv2.VideoCapture(video_filename)
while video.isOpened():
    ret, image = video.read()

    if not ret:
        break # we've reached the end of the video

    frames.append(image)



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

# go through the log array and video array to find the entries that correspond to each other
positionIndex = 0
frameIndex = 0
lastFrameIndex = None
frameCount = -1
combinedData = [] # a list of POSITIONs (or GPS coordinates) and corresponding CAMERA_IMAGEs

for idx, line in enumerate(log):
    entry = line.split(",") # it's a csv, so split along commas to get data

    match entry[0]:
        case "ENTRY_GPS":
            pass #TODO implement GPS stuff or something
        
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

        combinedData.append(Point(x, y, heading, lat, lon, image)) # then append the point

    
    lastFrameIndex = frameIndex

