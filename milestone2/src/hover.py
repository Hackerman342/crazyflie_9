#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from crazyflie_driver.msg import Position, Hover
from geometry_msgs.msg import PoseStamped, TransformStamped

from tf.transformations import quaternion_from_euler, euler_from_quaternion, inverse_matrix
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix




def pos_callback(msg):
    global x, y, z, yaw

    x=round(msg.pose.position.x,1)
    y=round(msg.pose.position.y,1)
    z=round(msg.pose.position.z,1)
    quat = msg.pose.orientation
    _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    yaw = round((180./np.pi)*yaw) # Convert yaw to degrees

def goal_callback(msg):
    global goal
    goal=msg
    print(goal)
    # goal=msg
    # goal.x=goal.x
    # goal.y=y+goal.y
    # goal.z=z+goal.z
    # goal.yaw=yaw+goal.yaw

# sub = rospy.Subscriber("cf1/pose", PoseStamped, pos_callback)
hover_publisher=rospy.Publisher("/cf1/cmd_position", Position, queue_size=10) #Publishes current height and position
hover_sub=rospy.Subscriber("goal", Position, goal_callback)



# Goal pose in map frame
# goal_matrix = translation_matrix((0.5, 0.5, 0)) # Initial position
# goal_pose=Position()
# goal_pose.x=0.5
# goal_pose.y=0.5
# goal_pose.z=0
# goal_pose.yaw=0
# Current_Position.header.frame_id = "map"

goal=None
rospy.sleep(2)

if __name__ == '__main__':
    global h, state
    rospy.init_node('hover')
    rospy.loginfo("Successful initilization of 'hover' node")

    odom_tf = TransformStamped()

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1)

    # Set initial goal to startup location in map frame (determined w/ loocalization node)
    try:
        init_tf = tf_buffer.lookup_transform("map", "cf1/base_link", rospy.Time.now(), rospy.Duration(10))
        # init_tf.transform.translation.z = 0.6
    except:
        print('Caught exception looking up map -> cf1/base_link tf')
        init_tf = TransformStamped()
        init_tf.transform.translation.x = 0.5
        init_tf.transform.translation.y = 0.5
        init_tf.transform.rotation.w = 1
        # init_tf.transform.translation.z = 0.6

    goal = Position()
    goal.x = init_tf.transform.translation.x
    goal.y = init_tf.transform.translation.y
    goal.z = 0.6
    init_quat = (init_tf.transform.rotation.x,
                init_tf.transform.rotation.y,
                init_tf.transform.rotation.z,
                init_tf.transform.rotation.w)
    _, _, yaw = euler_from_quaternion(init_quat)
    goal.yaw = math.degrees(yaw)

    print('Initial goal = starting position')
    print(goal)
    rospy.sleep(5)
    # height = init_tf.transform.translation.z

    # init_trans = (init_tf.transform.translation.x,
    #              init_tf.transform.translation.y,
    #              init_tf.transform.translation.z)

    # init_quat = (init_tf.transform.rotation.x,
    #              init_tf.transform.rotation.y,
    #              init_tf.transform.rotation.z,
    #              init_tf.transform.rotation.w)

    # tmat_init = translation_matrix(init_trans)
    # qmat_init = quaternion_matrix(init_quat)
    # goal_matrix = np.dot(tmat_init, qmat_init) # goal is initial pose until a goal is received

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():


        if goal:
            try:
                # Map in odom coords (order very important!!)
                odom_tf = tf_buffer.lookup_transform("cf1/odom", "map", rospy.Time.now())
            except:
                pass # Keep old transform
            odom_trans = (odom_tf.transform.translation.x,
                        odom_tf.transform.translation.y,
                        odom_tf.transform.translation.z)

            odom_quat = (odom_tf.transform.rotation.x,
                        odom_tf.transform.rotation.y,
                        odom_tf.transform.rotation.z,
                        odom_tf.transform.rotation.w)

            tmat_odom = translation_matrix(odom_trans)
            qmat_odom = quaternion_matrix(odom_quat)
            odom_matrix = np.dot(tmat_odom, qmat_odom)

            height = goal.z # Store/maintain given height
            # Goal in map coords
            tmat_goal = translation_matrix((goal.x, goal.y, goal.z))
            qmat_goal = quaternion_matrix(quaternion_from_euler(0, 0, math.radians(goal.yaw)))
            goal_matrix = np.dot(tmat_goal, qmat_goal)

            # Goal in odom coords
            final_mat = np.dot(odom_matrix, goal_matrix)
            trans = translation_from_matrix(final_mat)
            goal.x = trans[0]
            goal.y = trans[1]
            goal.z = height

            _, _, yaw = euler_from_quaternion(quaternion_from_matrix(final_mat))
            goal.yaw = math.degrees(yaw)
            # goal.header.frame_id = "cf1/odom"
            print('GOAL!!!!')
            print("Transformed goal 'cf1/odom'")
            print(goal)

            hover_publisher.publish(goal)
            goal_pose = goal
            goal=None

        else:
            # goal = Position()
            # # Goal in odom coords
            # final_mat = np.dot(odom_matrix, goal_matrix)
            # trans = translation_from_matrix(final_mat)
            # goal.x = trans[0]
            # goal.y = trans[1]
            # goal.z = height
            # _, _, yaw = euler_from_quaternion(quaternion_from_matrix(final_mat))
            # goal.yaw = yaw*(180/math.pi)
            # print('ELSE!!!!')
            # print("Transformed goal 'cf1/odom'")
            # # print(goal)
            # hover_publisher.publish(goal)
            # goal=None
            hover_publisher.publish(goal_pose)

        rate.sleep()
