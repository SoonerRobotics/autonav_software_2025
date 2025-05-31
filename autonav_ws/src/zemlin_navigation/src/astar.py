#!/usr/bin/env python3

from autonav_msgs.msg import Position, PathingDebug, SafetyLights, AudibleFeedback, WaypointReached
from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
import copy
from heapq import heappush, heappop
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import cv_bridge
import time
import threading
import os


GRID_SIZE = 0.1
VERTICAL_CAMERA_DIST = 2.75
HORIZONTAL_CAMERA_DIST = 3
CV_BRIDGE = cv_bridge.CvBridge()

class AStarConfig:
    def __init__(self):
        self.latitude_length = 111086.2
        self.longitude_length = 81978.2
        self.waypoint_pop_distance = 2.0
        self.waypoint_delay = 50
        self.robot_y = 66
        self.use_only_waypoints = False
        self.waypoint_sound = "~/autonav_software_2025/music/mine_xp.mp3"
        self.waypoints_started_sound = "~/autonav_software_2025/music/windows-xp-startup.mp3"
        self.waypoints = [
            # [(35.195074272, -97.438147936885), (35.1949329933, -97.43813450581), (35.19487062183, -97.43813631415), (35.1947369921, -97.43814289618)],

            # E-Quad path
            # [(35.21060116980733, -97.44233919102984), (35.21051527230819, -97.44233720628564), (35.21047672369589, -97.44231803913213), (35.2104757401181, -97.44212990887812), (35.21047600985816, -97.44192128767607), (35.21059801239906, -97.44209666880332)],

            # IGVC Qualification Points
            # [(42.668086, -83.218446)]

            # Single IGVC Waypoint
            # [(42.6679277, -83.2193276)]

            # The REAL (TM) QUAL POINT
            [(42.6683230, -83.2183619)]

            # IGVC Real Waypoints
            # [(42.6682623, -83.2193709), (42.6681206, -83.2193606), (42.6680766, -83.2193592), (42.6679277, -83.2193276), (42.6679216, -83.2189126), (42.668130236144883, -83.21889785301433)]
        ]

class AStarNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_astar")
        self.config = AStarConfig()
        self.reached_first_waypoint = False
        self.onReset()

    def apply_config(self, config):
        self.config.latitude_length = config["latitude_length"]
        self.config.longitude_length = config["longitude_length"]
        self.config.waypoint_pop_distance = config["waypoint_pop_distance"]
        self.config.waypoint_delay = config["waypoint_delay"]
        self.config.robot_y = config["robot_y"]
        self.config.use_only_waypoints = config["use_only_waypoints"]
        self.config.waypoints = config["waypoints"]
        # self.config.waypoint_sound = config["waypoint_sound"]

    def init(self):
        self.configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.onConfigSpaceReceived, 20)
        self.poseSubscriber = self.create_subscription(Position, "/autonav/position", self.onPoseReceived, 20)
        self.debugPublisher = self.create_publisher(PathingDebug, "/autonav/debug/astar", 20)
        self.pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.waypointReachedPublisher = self.create_publisher(WaypointReached, "/autonav/waypoint_reached", 20)
        self.bigSoundPublisher = self.create_publisher(AudibleFeedback, "/autonav/audible_feedback", 20)
        self.safetyLightsPublisher = self.create_publisher(SafetyLights, "/autonav/SafetyLights", 20)
        self.pathDebugImagePublisher = self.create_publisher(CompressedImage, "/autonav/path_debug_image", 20)
        self.nextDebugImage = None
        self.tick_timer = self.create_timer(0.05, self.createPath)
        self.resetWhen = -1.0
        self.set_device_state(DeviceState.OPERATING)

    def getAngleDifference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta
        
    def onReset(self):
        self.lastPath = None
        self.position = None
        self.configSpace = None
        self.reached_first_waypoint = False
        self.costMap = None
        self.bestPosition = (0, 0)
        self.waypoints = []
        self.waypointTime = 0.0

    def getWaypointsForDirection(self):
        return self.config.waypoints[0]

    def on_system_state_updated(self, old, new):
        if new == SystemState.AUTONOMOUS and self.mobility and len(self.waypoints) == 0:
            self.log(f"Waypoints will activate in {self.config.waypoint_delay} seconds")
            self.waypointTime = time.time() + self.config.waypoint_delay
            self.push_safety_lights(255, 255, 255, 1, 0)

        if new == SystemState.AUTONOMOUS and not self.mobility:
            self.push_safety_lights(255, 0, 130, 0, 0)
            
        if new != SystemState.AUTONOMOUS and self.get_device_state() == DeviceState.OPERATING:
            self.onReset()
            # self.push_safety_lights(255, 255, 255, 0, 0) - Handled in manual node

    def on_mobility_updated(self, old, new):
        if self.get_system_state() == SystemState.AUTONOMOUS and new and len(self.waypoints) == 0:
            self.log(f"Waypoints will activate in {self.config.waypoint_delay} seconds")
            self.waypointTime = time.time() + self.config.waypoint_delay
            self.push_safety_lights(255, 255, 255, 1, 0)
            
        if (self.get_system_state() != SystemState.AUTONOMOUS or not new) and self.get_device_state() == DeviceState.OPERATING:
            self.onReset()

        if old == True and new == False and self.get_system_state() == SystemState.AUTONOMOUS:
            self.push_safety_lights(255, 0, 0, 3, 0)
            
    def onPoseReceived(self, msg: Position):
        self.position = msg
        
    def createPath(self):
        if self.position is None or self.costMap is None:
            return
        
        robot_pos = (40, self.config.robot_y)
        path = self.findPathToPoint(robot_pos, self.bestPosition, self.costMap, 80, 80)
        if path is not None:
            global_path = Path()
            global_path.poses = [self.pathToGlobalPose(pp[0], pp[1]) for pp in path]
            self.lastPath = path
            self.pathPublisher.publish(global_path)
            self.nextDebugImage = path

    def playWaypointSound(self):
        audio = AudibleFeedback()
        audio.filename = os.path.expanduser(self.config.waypoint_sound)

        self.bigSoundPublisher.publish(audio)

    
    def play_started_waypointing_sound(self):
        audio = AudibleFeedback()
        audio.filename = os.path.expanduser(self.config.waypoints_started_sound)

        self.bigSoundPublisher.publish(audio)
    

    def createDebug(self):
        while True and rclpy.ok():
            path = self.nextDebugImage
            if self.nextDebugImage is not None and self.costMap is not None:
                # Draw the cost map onto a debug iamge
                cvimg = np.zeros((80, 80), dtype=np.uint8)
                for i in range(80):
                    for j in range(80):
                        cvimg[i, j] = self.costMap[ i * 80 + j ] * 255
                cvimg = cv2.cvtColor(cvimg, cv2.COLOR_GRAY2RGB)

                for pp in path:
                    cv2.circle(cvimg, (pp[0], pp[1]), 1, (0, 255, 0), 1)

                cv2.circle(cvimg, (self.bestPosition[0], self.bestPosition[1]), 1, (255, 0, 0), 1)
                
                cvimg = cv2.resize(cvimg, (800, 800), interpolation=cv2.INTER_NEAREST)
                self.pathDebugImagePublisher.publish(CV_BRIDGE.cv2_to_compressed_imgmsg(cvimg))
                self.nextDebugImage = None
            time.sleep(0.05)

    def reconstructPath(self, path, current):
        total_path = [current]

        while current in path:
            current = path[current]
            total_path.append(current)

        self.perf_stop("A*")
        return total_path[::-1]
        
    def findPathToPoint(self, start, goal, map, width, height):
        looked_at = np.zeros((80, 80))
        open_set = [start]
        path = {}
        search_dirs = []

        self.perf_start("A*")

        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue
                search_dirs.append((x,y,math.sqrt(x ** 2 + y ** 2)))

        def h(point):
            return math.sqrt((goal[0] - point[0]) ** 2 + (goal[1] - point[1]) ** 2)

        def d(to_pt, dist):
            return dist + map[to_pt[1] * width + to_pt[0]] / 10

        gScore = {}
        gScore[start] = 0

        def getG(pt):
            if pt in gScore:
                return gScore[pt]
            else:
                gScore[pt] = 1000000000
                return 1000000000 # Infinity

        fScore = {}
        fScore[start] = h(start)
        next_current = [(1,start)]
        while len(open_set) != 0:
            current = heappop(next_current)[1]

            looked_at[current[0],current[1]] = 1

            if current == goal:
                return self.reconstructPath(path, current)

            open_set.remove(current)
            for delta_x, delta_y, dist in search_dirs:

                neighbor = (current[0] + delta_x, current[1] + delta_y)
                if neighbor[0] < 0 or neighbor[0] >= width or neighbor[1] < 0 or neighbor[1] >= height:
                    continue

                tentGScore = getG(current) + d(neighbor, dist)
                if tentGScore < getG(neighbor):
                    path[neighbor] = current
                    gScore[neighbor] = tentGScore
                    fScore[neighbor] = tentGScore + h(neighbor)
                    if neighbor not in open_set:
                        open_set.append(neighbor)
                        heappush(next_current, (fScore[neighbor], neighbor))
                    
    def onConfigSpaceReceived(self, msg: OccupancyGrid):
        if self.position is None:
            return

        self.perf_start("Smellification")

        grid_data = msg.data
        temp_best_pos = (40, self.config.robot_y)
        best_pos_cost = -1000

        frontier = set()
        frontier.add((40, self.config.robot_y))
        explored = set()

        if self.config.use_only_waypoints == True:
            grid_data = [0] * len(msg.data)
            
        if len(self.waypoints) == 0 and time.time() > self.waypointTime and self.waypointTime != 0:
            self.waypoints = [wp for wp in self.getWaypointsForDirection()]
            self.waypointTime = 0
            self.push_safety_lights(255, 255, 0, 1, 2)
            self.push_safety_lights(255, 255, 255, 1, 0)
            self.play_started_waypointing_sound()
        
        if time.time() < self.waypointTime and len(self.waypoints) == 0:
            time_remaining = self.waypointTime - time.time()
            pathingDebug = PathingDebug()
            pathingDebug.waypoints = []
            pathingDebug.time_until_use_waypoints = time_remaining
            self.debugPublisher.publish(pathingDebug)

        if time.time() > self.resetWhen and self.resetWhen != -1 and self.mobility:
            # self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 255, "#FFFFFF"))
            self.resetWhen = -1

        if len(self.waypoints) > 0:
            next_waypoint = self.waypoints[0]
            north_to_gps = (next_waypoint[0] - self.position.latitude) * self.config.latitude_length
            west_to_gps = (self.position.longitude - next_waypoint[1]) * self.config.longitude_length
            heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            if north_to_gps ** 2 + west_to_gps ** 2 <= self.config.waypoint_pop_distance:
                if self.reached_first_waypoint:
                    self.push_safety_lights(0, 255, 0, 1, 2)
                else:
                    self.push_safety_lights(0, 0, 255, 1, 2)
                    self.reached_first_waypoint = True
                self.push_safety_lights(255, 255, 255, 1, 0)
                self.waypoints.pop(0)
                # self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 255, "#00FF00"))
                self.resetWhen = time.time() + 1.5
                self.waypointReachedPublisher.publish(WaypointReached(
                    latitude=next_waypoint[0],
                    longitude=next_waypoint[1],
                    tag="peepeepoopoo"
                ))
                # self.playWaypointSound()

            pathingDebug = PathingDebug()
            pathingDebug.desired_heading = heading_to_gps
            pathingDebug.desired_latitude = next_waypoint[0]
            pathingDebug.desired_longitude = next_waypoint[1]
            pathingDebug.distance_to_destination = north_to_gps ** 2 + west_to_gps ** 2
            wp1d = []
            for wp in self.waypoints:
                wp1d.append(wp[0])
                wp1d.append(wp[1])
            pathingDebug.waypoints = wp1d
            self.debugPublisher.publish(pathingDebug)

        depth = 0
        while depth < 50 and len(frontier) > 0:
            curfrontier = copy.copy(frontier)
            for pos in curfrontier:
                x = pos[0]
                y = pos[1]
                cost = (80 - y) * 1.3 + depth * 2.2

                if len(self.waypoints) > 0:
                    heading_err_to_gps = abs(self.getAngleDifference(self.position.theta + math.atan2(40 - x, 80 - y), heading_to_gps)) * 180 / math.pi
                    cost -= max(heading_err_to_gps, 10)
                    # cost -= max(heading_err_to_gps, 30)

                if cost > best_pos_cost:
                    best_pos_cost = cost
                    temp_best_pos = pos

                frontier.remove(pos)
                explored.add(x + 80 * y)

                if y > 1 and grid_data[x + 80 * (y-1)] < 50 and x + 80 * (y-1) not in explored:
                    frontier.add((x, y - 1))

                if x < 79 and grid_data[x + 1 + 80 * y] < 50 and x + 1 + 80 * y not in explored:
                    frontier.add((x + 1, y))

                if x > 0 and grid_data[x - 1 + 80 * y] < 50 and x - 1 + 80 * y not in explored:
                    frontier.add((x - 1, y))

            depth += 1

        self.costMap = grid_data
        self.bestPosition = temp_best_pos
        self.perf_stop("Smellification", True)
        
    def pathToGlobalPose(self, pp0, pp1):
        x = (80 - pp1) * VERTICAL_CAMERA_DIST / 80
        y = (40 - pp0) * HORIZONTAL_CAMERA_DIST / 80
        
        new_x = x * math.cos(0) + y * math.sin(0) 
        new_y = x * math.sin(0) + y * math.cos(0)
        pose = PoseStamped()
        point = Point()
        point.x = new_x
        point.y = new_y
        pose.pose.position = point
        return pose


def main():
    rclpy.init()
    rclpy.spin(AStarNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
