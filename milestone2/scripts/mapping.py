#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import numpy as np
import sys
import rospy
from std_srvs.srv import SetBool
from rospy.numpy_msg import numpy_msg

# todo create a publisher that sends the map matrix
# todo fix the expansion, centralize the origin when expansion is applied
# todo fix the padding, perhaps draw a box around instead of adding "walls" in every direction

# zm is only a placeholder

class Mapping:

    def __init__(self, json_file_dir, step_size, inflation, expansion=0):
        # json_file_dir is the directory where the json file is located
        # step size is how small each grid will be. 0.1 is 10 cm, which is what I used
        # inflation is how big you want the walls to be, needs fixing
        # expansion does not work yet, it expands the map

        self.jfile = json_file_dir
        self.step = step_size
        self.infl = inflation

        with open(self.jfile) as jfile:
            self.map_data = json.load(jfile)
            map_end, map_start = self.airspace()

            if expansion > 0:
                map_start, map_end = self.expand(map_start, map_end, expansion)

            self.matrix = self.clean_map(map_start, map_end, 0, self.step)
            # x_length = len(self.matrix[:,0])
            # y_length = len(self.matrix[0,:])
            self.x_conv = abs(map_start[0])/step_size
            self.y_conv = abs(map_start[1])/step_size

            for wall in self.map_data["walls"]:
                wall_start = wall["plane"]["start"]
                wall_stop = wall["plane"]["stop"]
                points = self.line(wall_start[:2], wall_stop[:2])
                self.add_objects(points, wall=True)

            self.objects(markers=True)
            self.objects(roadsigns=True)


    def airspace(self):
        # Extracting the boundries for the airspace
        max_coords = self.map_data["airspace"]["max"]
        min_coords = self.map_data["airspace"]["min"]
        return max_coords, min_coords

    def expand(self, start, end, exp):
        exp_start = (start[1] + exp, start[0] + exp)
        exp_end = (end[1] + exp, end[0] + exp)
        return exp_start, exp_end

    def objects(self, markers=False, roadsigns=False):
        # Adding objects to the map matrix
        if markers == True:
            size = self.map_data["marker_size"]
        elif roadsigns == True:
            size = self.map_data["roadsign_size"]
        else:
            # Raise error msg later perhaps?
            return 0

        # width = (size[1]/2)/self.step
        # height = (size[0]/2)/self.step
        points = []
        
        # Below is for 
        # xm = xm/self.step
        # ym = ym/self.step
        
        if markers == True:    
            for marker in self.map_data["markers"]:
                (xm, ym, zm) = marker["pose"]["position"]
                point = (int(xm/self.step), int(ym/self.step))
                points.append(point)
            self.add_objects(points, marker=True)

        elif roadsigns == True:
            for marker in self.map_data["roadsigns"]:
                (xm, ym, zm) = marker["pose"]["position"]
                point = (int(xm/self.step), int(ym/self.step))
                points.append(point)
            self.add_objects(points, roadsign=True)

        else:
            # Raise error msg later?
            return 0
        

    def clean_map(self, start, end, expansion, step_size):
        # Generating a zero matrix for the entire map + some extending it a bit
        # since markers could be outside the airspace
        x = (end[0] + abs(start[0]) + expansion)/step_size
        y = (end[1] + abs(start[1]) + expansion)/step_size
        map_matrix = np.zeros((int(x), int(y)))
        return map_matrix
       
    def inflate_walls(self, xidx, yidx):
        # inflating the walls so they appear larger in the map
        yidx = int(yidx)
        xidx = int(xidx)
        for i in range(self.infl):
            
            self.matrix[yidx, xidx + i] = 1
            self.matrix[yidx + i, xidx + i] = 1
            self.matrix[yidx + i, xidx] = 1
            self.matrix[yidx + i, xidx - i] = 1
            self.matrix[yidx, xidx - i] = 1
            self.matrix[yidx - i, xidx - i] = 1
            self.matrix[yidx - i, xidx] = 1
            self.matrix[yidx - i, xidx + i] = 1
    
    def line(self, start, end):
        """
        Code of honor: Algorithm from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python
        """
        x1, y1 = start
        x2, y2 = end
        x1 = x1/self.step
        x2 = x2/self.step
        y1 = y1/self.step
        y2 = y2/self.step
        dx = x2 - x1
        dy = y2 - y1


        is_steep = abs(dy) > abs(dx)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        
        dx = x2 - x1
        dy = y2 - y1

        error = int(dx/2)
        ystep = 1 if y1 < y2 else -1

        y = y1
        points = []

        for x in range(int(x1), int(x2) + 1):
            coord = (y, x) if is_steep else(x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        
        if swapped:
            points.reverse()
        
        return points

    def add_objects(self, points, wall=False, marker=False, roadsign=False):
        # adding the assigned objects into the map matrix
        for p in points:

            yidx = self.y_conv + p[1]
            xidx = self.x_conv + p[0]
            
            # yidx = conv[0] - p[0]
            # xidx = conv[1] + p[1]
            p_shift = (int(yidx), int(xidx))
            if wall == True:
                self.matrix[p_shift] = 1
                self.inflate_walls(xidx, yidx)
            elif marker == True:
                self.matrix[p_shift] = 4
            elif roadsign == True:
                self.matrix[p_shift] = 5
    
    def wall_values(self):
        pass

    def object_poses(self):
        # Create a list of lists containing the poses of the markers and signs.
        # Returns two lists with tuples. First element in the tuple is the marker id or roadsign name
        # Second element in the list is a list with the pose. [x y z angles] 
        objects = {}

        markers = []
        signs = []

        for object in self.map_data["markers"]:
            # Place holder for name or ID if needed.
            name = object["id"]
            pose = object["pose"]["position"] + object["pose"]["orientation"]
            
            objects[name] = pose

            markers.append((name, pose))

        for object in self.map_data["roadsigns"]:
            # Place holder for name or ID if needed.
            name = object["sign"]
            pose = object["pose"]["position"] + object["pose"]["orientation"]
            
            objects[name] = pose

            signs.append((name, pose))
        
        return markers, signs, objects

    
"""
def create_map(x):
    # These are hard coded values, might change in the future
    json_dir = "awesome.world.json"
    step_size = 0.1
    inflation = 2
    mapp = Mapping(json_dir, step_size, inflation)
    pass

def map_server():
    rospy.init_node("create_map_service")
    s = rospy.Service("create_map", SetBool, create_map)
    rospy.spin()

if __name__ == "__main__":
    map_server()
"""