#!/usr/bin/env python

import sys
import rospy
from raspberry_screen.srv import DisplayScreen

def display_screen_client(message):
    rospy.wait_for_service('display_screen')
    try:
        display_image = rospy.ServiceProxy('display_screen', DisplayScreen)
        return "Result: ",display_image(message).result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [emotion]"%sys.argv[0]

if __name__ == "__main__":

    try:
        if len(sys.argv) > 1:
            message = sys.argv[len(sys.argv)-1]
    except:
        rospy.loginfo("Can not use arg.")
        sys.exit(1)
    print "Requesting %s"%message
    print display_screen_client(message)
