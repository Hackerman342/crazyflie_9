#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from crazyflie_driver.msg import Position
from find_path import * # modify the start point and end point in find_path.py file

def publish_path():
    # pathx = [74.0, 76.0, 78.0, 80.0, 82.0, 84.0, 86.0, 88.0, 90.0, 92.0, 94.0, 96.0, 98.0, 100.0, 102.0, 104.0, 106.0, 108.0, 110.0, 112.0, 114.0, 116.0, 118.0, 120.0]
    # pathy = [100.0, 100.0, 100.0, 102.0, 102.0, 104.0, 106.0, 106.0, 106.0, 108.0, 108.0, 110.0, 110.0, 110.0, 112.0, 112.0, 114.0, 114.0, 116.0, 116.0, 118.0, 118.0, 118.0, 120.0]
    rospy.init_node('path_following', anonymous=True)
    rx,ry = find_path()
    # print(rx)
    # print(ry)
    pathx = [c/100 for c in rx]
    pathy = [c/100 for c in ry]
    pathz = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9]
    pathyaw = [0]*20
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = '10' # a simple random number


    pub = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        for i in range(20):
            cmd.x = pathx[i]
            cmd.y = pathy[i]
            cmd.z = pathz[i]
            cmd.yaw = pathyaw[i]
            pub.publish(cmd)
            rate.sleep()
        for j in range(20):
            cmd.x = pathx[19-j]
            cmd.y = pathy[19-j]
            cmd.z = pathz[19-j]
            cmd.yaw = pathyaw[19-j]
            pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass