#!/usr/bin/env python

import sys
import rospy

from full_system.srv import PathPlanning, PathPlanningResponse


def path_planning_client(start_x, start_y, end_x, end_y):
    rospy.wait_for_service('path_planning')
    try:
        path_planning = rospy.ServiceProxy('path_planning', PathPlanning)
        print("Here is OK")
        path = path_planning(start_x, start_y, end_x, end_y) # request message
        return path
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    # Given when you call the client node
    if len(sys.argv) == 5:
        start_x = int(sys.argv[1])
        start_y = int(sys.argv[2])
        end_x = int(sys.argv[3])
        end_y = int(sys.argv[4])
    else:
        print usage()
        sys.exit(1)

    print "From (%s,%s) to (%s,%s), the path is"%(start_x, start_y, end_x, end_y)

    path = path_planning_client(start_x, start_y, end_x, end_y) # request message
    print(path)
