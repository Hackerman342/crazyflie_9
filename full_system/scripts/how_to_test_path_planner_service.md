Without gazebo:

1. run roscore
2. go to /dd2419_ws/src/crazyflie_9/milestone3/scripts run python3 path_planning_server.py
3. go to /dd2419_ws/src/crazyflie_9/milestone3/scripts run python3 path_following.py; In this step, you can see the rx and ry in this terminal window
4. run rostopic echo /cf1/cmd_position; where you can see the x, y, z, yaw

With gazebo:

1. run roscore
2. run roslaunch dd2419_simulation simulation.launch gui:=false
3. run rviz
4. go to /dd2419_ws/src/crazyflie_9/milestone3/scripts run python3 path_planning_server.py
5. go to /dd2419_ws/src/crazyflie_9/milestone3/scripts run python3 path_following.py; In this step, you can see the rx and ry in this terminal window

Results:
It will go up first and then go down

Problem:
Have to all use python3 to run the python file rather than ordinary rosrun way

rospy.service.ServiceException: service [/path_planning] responded with an error: b'error processing request: range() integer end argument expected, got float.'
