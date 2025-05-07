#!/usr/bin/env python3

from autonav_msgs.msg import Position
from autonav_shared.node import Node
from autonav_shared.types import DeviceState, SystemState
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
import copy
from heapq import heappush, heappop
import numpy as np
import cv2
import cv_bridge
import time


GRID_SIZE = 0.1
VERTICAL_CAMERA_DIST = 2.75
HORIZONTAL_CAMERA_DIST = 3
CV_BRIDGE = cv_bridge.CvBridge()

simulation_waypoints = [
    # [(35.19510000, -97.43895000), (35.19491000, -97.43896000), (35.1948357, -97.43896), (35.19467540, -97.43895)],  # NORTH
    # [(35.19467540, -97.43895), (35.1948357, -97.43896), (35.19491000, -97.43896000), (35.19510000, -97.43895000)],  # SOUTH
    # [(35.194725, -97.43858), (35.1947823, -97.4387), (35.1948547, -97.43876), (35.1949272, -97.43867), (35.1950035, -97.43881)], # PRACTICE
    [(35.19506, -97.43824), (35.19491, -97.43824), (35.19484, -97.43824), (35.19472, -97.43823)] 
]


class AStarConfig:
    def __init__(self):
        self.waypoint_pop_distance = 1.1
        self.waypoint_direction = 0
        self.use_only_waypoints = False
        self.waypoint_delay = 1699.5
        self.latitude_length = 111086.2
        self.longitude_length = 81978.2


class AStarNode(Node):
    def __init__(self):
        super().__init__("zemlin_astar")
        self.config = AStarConfig()
        self.on_reset()

    def init(self):
        self.configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.cfg_space_received, 20)
        self.poseSubscriber = self.create_subscription(Position, "/autonav/position", self.on_position_received, 20)
        self.pathDebugImagePublisher = self.create_publisher(CompressedImage, "/autonav/path_debug_image", 20)
        self.pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.mapTimer = self.create_timer(0.1, self.create_path)
        self.resetWhen = -1.0

    def apply_config(self, config):
        self.config.waypoint_pop_distance = config["waypoint_pop_distance"]
        self.config.waypoint_direction = config["waypoint_direction"]
        self.config.use_only_waypoints = config["use_only_waypoints"]
        self.config.waypoint_delay = config["waypoint_delay"]
        self.config.latitude_length = config["latitude_length"]
        self.config.longitude_length = config["longitude_length"]

    def angle_diff(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta
        
    def on_reset(self):
        self.lastPath = None
        self.position = None
        self.configSpace = None
        self.costMap = None
        self.bestPosition = (0, 0)
        self.waypoints = []
        self.waypointTime = 0.0

    def get_waypoints(self):
        return simulation_waypoints[0]

    def on_system_state_updated(self, old, new):
        if new == SystemState.AUTONOMOUS and self.is_mobility and len(self.waypoints) == 0:
            self.waypointTime = time.time() + self.config.waypoint_delay
        
        if new != SystemState.AUTONOMOUS and self.device_states.get(self.get_name()) == DeviceState.OPERATING:
            self.on_reset()
            
    def on_position_received(self, msg: Position):
        self.position = msg
        
    def create_path(self):
        if self.position is None or self.costMap is None:
            return
        
        robot_pos = (40, 78)
        path = self.find_path_to_point(robot_pos, self.bestPosition, self.costMap, 80, 80)
        if path is not None:
            global_path = Path()
            global_path.poses = [self.pathToGlobalPose(pp[0], pp[1]) for pp in path]
            self.lastPath = path
            self.pathPublisher.publish(global_path)

            # # Draw the cost map onto a debug iamge
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
            
    def reconstruct(self, path, current):
        total_path = [current]

        while current in path:
            current = path[current]
            total_path.append(current)

        return total_path[::-1]
        
    def find_path_to_point(self, start, goal, map, width, height):
        looked_at = np.zeros((80, 80))
        open_set = [start]
        path = {}
        search_dirs = []

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
                return self.reconstruct(path, current)

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
                    
    def cfg_space_received(self, msg: OccupancyGrid):
        if self.position is None or self.system_state != SystemState.AUTONOMOUS:
            return

        grid_data = msg.data
        temp_best_pos = (40, 78)
        best_pos_cost = -1000

        frontier = set()
        frontier.add((40, 78))
        explored = set()

        if self.config.use_only_waypoints:
            grid_data = [0] * len(msg.data)
            
        if len(self.waypoints) == 0 and time.time() > self.waypointTime and self.waypointTime != 0:
            self.waypoints = [wp for wp in self.get_waypoints()]
            self.waypointTime = 0

        if time.time() > self.resetWhen and self.resetWhen != -1 and self.is_mobility():
            self.resetWhen = -1

        if len(self.waypoints) > 0:
            next_waypoint = self.waypoints[0]
            north_to_gps = (next_waypoint[0] - self.position.latitude) * self.config.latitude_length
            west_to_gps = (self.position.longitude - next_waypoint[1]) * self.config.longitude_length
            heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            if north_to_gps ** 2 + west_to_gps ** 2 <= self.config.waypoint_pop_distance:
                self.waypoints.pop(0)
                # self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 255, "#00FF00"))
                self.resetWhen = time.time() + 1.5

            # pathingDebug = PathingDebug()
            # pathingDebug.desired_heading = heading_to_gps
            # pathingDebug.desired_latitude = next_waypoint[0]
            # pathingDebug.desired_longitude = next_waypoint[1]
            # pathingDebug.distance_to_destination = north_to_gps ** 2 + west_to_gps ** 2
            # wp1d = []
            # for wp in self.waypoints:
            #     wp1d.append(wp[0])
            #     wp1d.append(wp[1])
            # pathingDebug.waypoints = wp1d
            # self.debugPublisher.publish(pathingDebug)

        depth = 0
        while depth < 50 and len(frontier) > 0:
            curfrontier = copy.copy(frontier)
            for pos in curfrontier:
                x = pos[0]
                y = pos[1]
                cost = (80 - y) * 1.3 + depth * 2.2

                if len(self.waypoints) > 0:
                    heading_err_to_gps = abs(self.angle_diff(self.position.theta + math.atan2(40 - x, 80 - y), heading_to_gps)) * 180 / math.pi
                    cost -= max(heading_err_to_gps, 10)

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