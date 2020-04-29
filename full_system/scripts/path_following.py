#!/usr/bin/env python

import rospy
# import sys
from std_msgs.msg import String
from crazyflie_driver.msg import Position
from find_path import * # modify the start point and end point in find_path.py file
from full_system.srv import PathPlanning, PathPlanningResponse

def path_planning_client(start_x, start_y, end_x, end_y):
    rospy.wait_for_service('path_planning')
    # try:
    path_planning = rospy.ServiceProxy('path_planning', PathPlanning)
    path = path_planning(start_x, start_y, end_x, end_y) # request message
    return path
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e


def publish_path():

    rospy.init_node('path_following', anonymous=True)
    start_x = 120.0  # [m]
    start_y = 100.0  # [m]
    end_x = 20.0  # [m]
    end_y = 100.0  # [m]
    path = path_planning_client(start_x, start_y, end_x, end_y) # request message
    print(path) # rx:[....]\n ry:[....]

    pathx = [c*0.05 for c in path.rx] # tranfer from pixels to meters
    pathy = [c*0.05 for c in path.ry]
    pathz = [0.6]*len(pathx)
    pathyaw = [0]*len(pathx)
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = '10' # a simple random number


    pub = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        for i in range(len(pathx)):
            cmd.x = pathx[i]
            cmd.y = pathy[i]
            cmd.z = pathz[i]
            cmd.yaw = pathyaw[i]
            pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':

    publish_path()