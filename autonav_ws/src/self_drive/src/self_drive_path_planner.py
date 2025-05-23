import copy
from heapq import heappop, heappush
import math

import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path

GRID_SIZE = 0.1
VERTICAL_CAMERA_DIST = 2.75
HORIZONTAL_CAMERA_DIST = 3

class SelfDrivePathPlanner:
    def __init__(self):
        self.costMap = None

    def plan(self, msg: OccupancyGrid):
        grid_data = msg.data
        temp_best_pos = (40, 68)
        best_pos_cost = -1000

        frontier = set()
        frontier.add((40, 68))
        explored = set()
        
        depth = 0
        while depth < 50 and len(frontier) > 0:
            curfrontier = copy.copy(frontier)
            for pos in curfrontier:
                x = pos[0]
                y = pos[1]
                cost = (80 - y) * 1.3 + depth * 2.2

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

        return self.create_path()

    def create_path(self):
        if self.costMap is None:
            return

        robot_pos = (40, 68)
        path = self.find_path_to_point(robot_pos, self.bestPosition, self.costMap, 80, 80)
        if path is not None:
            global_path = Path()
            global_path.poses = [self.pathToGlobalPose(pp[0], pp[1]) for pp in path]
            self.lastPath = path
            
            return global_path
        
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
                    

    def reconstruct(self, path, current):
        total_path = [current]

        while current in path:
            current = path[current]
            total_path.append(current)

        return total_path[::-1]
    
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
