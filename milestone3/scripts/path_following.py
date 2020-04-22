#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
# import sys
from std_msgs.msg import String
from crazyflie_driver.msg import Position
from find_path import * # modify the start point and end point in find_path.py file
from milestone3.srv import PathPlanning, PathPlanningResponse

def path_planning_client(start_x, start_y, end_x, end_y):
    rospy.wait_for_service('path_planning')
    # try:
    path_planning = rospy.ServiceProxy('path_planning', PathPlanning)
    path = path_planning(start_x, start_y, end_x, end_y) # request message
    return path
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e


def publish_path():
    # pathx = [74.0, 76.0, 78.0, 80.0, 82.0, 84.0, 86.0, 88.0, 90.0, 92.0, 94.0, 96.0, 98.0, 100.0, 102.0, 104.0, 106.0, 108.0, 110.0, 112.0, 114.0, 116.0, 118.0, 120.0]
    # pathy = [100.0, 100.0, 100.0, 102.0, 102.0, 104.0, 106.0, 106.0, 106.0, 108.0, 108.0, 110.0, 110.0, 110.0, 112.0, 112.0, 114.0, 114.0, 116.0, 116.0, 118.0, 118.0, 118.0, 120.0]
    rospy.init_node('path_following', anonymous=True)
    start_x = 120.0  # [m]
    start_y = 100.0  # [m]
    end_x = 20.0  # [m]
    end_y = 100.0  # [m] 
    # rx,ry = find_path(start_x,start_y,end_x,end_y)
    path = path_planning_client(start_x, start_y, end_x, end_y) # request message
    print(path) # rx:[....]\n ry:[....]
    # print(path.rx) # (xx, xx, xx)
    # print(type(path.rx)) #<class 'tuple'>
    # for i in range(5):
    #     print((path.rx[i]))
    # print(rx)
    # print(ry)
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
        # for j in range(20):
        #     cmd.x = pathx[19-j]
        #     cmd.y = pathy[19-j]
        #     cmd.z = pathz[19-j]
        #     cmd.yaw = pathyaw[19-j]
        #     pub.publish(cmd)
        #     rate.sleep()

if __name__ == '__main__':
    # if len(sys.argv) == 5:
    #     start_x = int(sys.argv[1])
    #     start_y = int(sys.argv[2])
    #     end_x = int(sys.argv[3])
    #     end_y = int(sys.argv[4])
    # start_x = 75.0  # [m]
    # start_y = 100.0  # [m]
    # end_x = 120.0  # [m]
    # end_y = 120.0  # [m]
    # path = path_planning_client(start_x, start_y, end_x, end_y)
    publish_path()