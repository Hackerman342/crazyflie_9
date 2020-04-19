#!/usr/bin/env python
import rospy
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def publish_path():
    rospy.init_node('nav_msgs_path_test')
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    path = Path()
    counter = 0
    p = PoseStamped()
    
    p.pose.position.x = 3
    p.pose.position.y = 0.0
    p.pose.position.z = 0.22
    
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1
    # q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
    # p.pose.orientation = Quaternion(*q)
    rate = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = str(counter)
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = str(counter)
        path.poses.append(p)
        rospy.loginfo('I have %d', path.poses[0].pose.position.x)
        path_pub.publish(path)
        counter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass