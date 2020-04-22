#!/usr/bin/env python

from a_star_algorithm import *
def find_path(sx, sy, gx, gy):
    # sx = 75.0  # [m]
    # sy = 100.0  # [m]
    # gx = 120.0  # [m]
    # gy = 120.0  # [m]

    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]
    location = '/home/robot/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json'
    # mapp = Mapping(location, 0.1, 3)
    mapp = Mapping(location, 0.05, 2)
    matrx = mapp.matrix
    range_of_map = matrx.shape
    # x_conv = mapp.x_conv

    horizonal = range_of_map[0]
    vertical = range_of_map[1]
    # print(matrx.shape)
    # print(horizonal)
    # print(vertical)
    # set obstable positions
    matrx_indx = np.nonzero(matrx == 1) # represent the walls
    oy_old = matrx_indx[0].tolist()
    ox_old = matrx_indx[1].tolist()
    oy = [vertical-i for i in oy_old]
    ox = [horizonal-i for i in ox_old]
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    rx.reverse()
    ry.reverse()
    # rx_new = [i-mapp.x_conv for i in rx]
    # ry_new = [i-mapp.y_conv for i in ry]
    return rx,ry
    # return rx,ry