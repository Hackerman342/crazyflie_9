#!/usr/bin/env python

import math
import rospy
#from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# import tf2_ros
# import tf2_geometry_msgs
from crazyflie_driver.msg import Position


class FlyAndSpin():

    def __init__(self):
        # Pull ROS parameters from launch file:
        param = rospy.search_param("goal_topic")
        self.goal_topic = rospy.get_param(param)
        param = rospy.search_param("goal_frame")
        self.goal_frame = rospy.get_param(param)
        param = rospy.search_param("sim_or_real")
        self.sim_or_real = rospy.get_param(param)
        param = rospy.search_param("distance_forward")
        self.distance_forward = rospy.get_param(param)
        param = rospy.search_param("spin_count")
        self.spin_count = rospy.get_param(param)

        # Confirm sim or real is proper
        if not (self.sim_or_real == 'sim' or self.sim_or_real == 'real'):
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                rospy.loginfo("sim_or_real mmust be set to either 'sim' or 'real'")
                rate.sleep()


        
        # Initialize goal message and publisher
        self.goal = Position()
        self.goal.header.frame_id = self.goal_frame
        self.pub_goal  = rospy.Publisher(self.goal_topic, Position, queue_size=10)
        # Delay briefly for publisher to initialize
        rospy.sleep(1)


    def goal_build(self, x, y, z, yaw):
        # Position in [m] relative to odom origin
        self.goal.x = x
        self.goal.y = y
        self.goal.z = z
        # Orientationin [deg] relative to odom origin
        self.goal.yaw = yaw # About z-axis


    def flight_sequence(self):
        # Set delay times to enable goal types to be reached
        up_delay = 5
        forward_delay = 8
        spin_delay = 1
        ## 1. Fly vertically up
        # Build goal directly above
        if self.sim_or_real == 'sim':
            x0, y0 = 0, 0
        else:
            x0, y0 = 0.5, 0.5
        self.goal_build(x0, y0, 0.4, 0)
        # Send goal
        self.pub_goal.publish(self.goal)
        rospy.loginfo("Goal directly above sent")
        rospy.sleep(up_delay) # Delay __ seconds to ensure drone made it to goal
        
        ## 2. Fly straight ahead X meters
        # Build goal straight ahead
        self.goal_build(x0 + self.distance_forward, y0, 0.4, 0)
 
        # Send goal
        self.pub_goal.publish(self.goal)
        rospy.loginfo("Goal straight ahead sent")
        rospy.sleep(forward_delay) # Delay __ seconds to ensure drone made it to goal

        ## 3. Spin around N times
        for i in range(self.spin_count):
            # Build spin half goal straight ahead
            self.goal_build(x0 + self.distance_forward, y0, 0.4, 90)
            # Send goal
            self.pub_goal.publish(self.goal)
            rospy.loginfo("Goal spin to 90 sent")
            rospy.sleep(spin_delay) # Delay __ seconds to ensure drone made it to goal
            
            # Build spin half goal straight ahead
            self.goal_build(x0 + self.distance_forward, y0, 0.4, 180)
            # Send goal
            self.pub_goal.publish(self.goal)
            rospy.loginfo("Goal spin to 180 sent")
            rospy.sleep(spin_delay) # Delay __ seconds to ensure drone made it to goal
            
            # Build spin half goal straight ahead
            self.goal_build(x0 + self.distance_forward, y0, 0.4, 270)
            # Send goal
            self.pub_goal.publish(self.goal)
            rospy.loginfo("Goal spin to 270 sent")
            rospy.sleep(spin_delay) # Delay __ seconds to ensure drone made it to goal

            # Build spin half goal straight ahead
            self.goal_build(x0 + self.distance_forward, y0, 0.4, 0)
            # Send goal
            self.pub_goal.publish(self.goal)
            rospy.loginfo("Goal spin to 0 sent")
            rospy.sleep(spin_delay) # Delay __ seconds to ensure drone made it to goal
        
        ## 4. Face other direction
        # Build spin half goal straight ahead
        self.goal_build(x0 + self.distance_forward, y0, 0.4, 180)
        # Send goal
        self.pub_goal.publish(self.goal)
        rospy.loginfo("Goal face back to origin")
        rospy.sleep(2*spin_delay) # Delay __ seconds to ensure drone made it to goal

        ## 5. Fly back to origin
        # Build goal directly above
        self.goal_build(x0, y0, 0.4, 180)
        # Send goal
        self.pub_goal.publish(self.goal)
        rospy.loginfo("Goal back to origin sent")
        rospy.sleep(forward_delay) # Delay __ seconds to ensure drone made it to goal



if __name__ == '__main__':
    rospy.sleep(3) # Delay __ seconds to ensure hover node is running
    
    rospy.init_node('fly_and_spin', anonymous=True)
    rospy.loginfo("Successful initilization of 'fly_and_spin' node")
    
    fs = FlyAndSpin()
    rospy.loginfo("Successful execution of init function")
    
    fs.flight_sequence()
    
    rospy.loginfo("Successfully ran script")

