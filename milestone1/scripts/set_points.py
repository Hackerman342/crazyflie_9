#!/usr/bin/env python

import math
import rospy
#from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# import tf2_ros
# import tf2_geometry_msgs
from crazyflie_driver.msg import Position


class GoalPublisher():

    def __init__(self):
        # Pull ROS parameters from launch file:
        param = rospy.search_param("goal_topic")
        self.goal_topic = rospy.get_param(param) # self.goal_topic = goal
        param = rospy.search_param("goal_frame")
        self.goal_frame = rospy.get_param(param)
        param = rospy.search_param("goal_pose")
        self.goal_pose_string = rospy.get_param(param)
        self.goal_pose_list = list(self.goal_pose_string.split(", "))
        self.goal_pose = [float(i) for i in self.goal_pose_list]


        # Initialize goal message and publisher
        self.goal = Position()
        self.goal.header.frame_id = self.goal_frame
        self.pub_goal  = rospy.Publisher(self.goal_topic, Position, queue_size=10)
        # Delay briefly for publisher to initialize
        rospy.sleep(.3)


    def goal_build(self, x, y, z, yaw):
        # Position in [m] relative to odom origin
        self.goal.x = x
        self.goal.y = y
        self.goal.z = z
        # Orientationin [deg] relative to odom origin
        self.goal.yaw = yaw # About z-axis


    def goal_pub(self):
        # Build goal for stop sign detection (awesome map)
        g = self.goal_pose
        self.goal_build(g[0], g[1], g[2], g[3])
        if self.goal.z == 0.0:
            self.goal.z = 0.4
            # Send goal
        self.pub_goal.publish(self.goal)
        rospy.loginfo("New goal sent")



if __name__ == '__main__':
    rospy.init_node('goal_pub', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    goalpub = GoalPublisher()
    rospy.loginfo("Successful execution of init function")

    runtime = 2 # seconds
    pubrate = 2

    rate = rospy.Rate(pubrate) # hz
    for i in range(runtime*pubrate):
        goalpub.goal_pub()
        rate.sleep()

    rospy.loginfo("Successfully ran script")

    # def publish_cmd(goal):
    #     # Need to tell TF that the goal was just generated
    #     goal.header.stamp = rospy.Time.now()

    #     if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', goal.header.stamp):
    #         rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
    #         return

    #     goal_odom = tf_buf.transform(goal, 'cf1/odom')

    #     cmd = Position()

    #     cmd.header.stamp = rospy.Time.now()
    #     cmd.header.frame_id = goal_odom.header.frame_id

    #     cmd.x = goal_odom.pose.position.x
    #     cmd.y = goal_odom.pose.position.y
    #     cmd.z = goal_odom.pose.position.z

    #     roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
    #                                             goal_odom.pose.orientation.y,
    #                                             goal_odom.pose.orientation.z,
    #                                             goal_odom.pose.orientation.w))

    #     cmd.yaw = math.degrees(yaw)

    #     pub_cmd.publish(cmd)


    # rospy.init_node('navgoal3')
    # sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    # pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    # tf_buf   = tf2_ros.Buffer()
    # tf_lstn  = tf2_ros.TransformListener(tf_buf)

    # def main():
    #     rate = rospy.Rate(10)  # Hz
    #     while not rospy.is_shutdown():
    #         if goal:
    #             publish_cmd(goal)
    #         rate.sleep()

