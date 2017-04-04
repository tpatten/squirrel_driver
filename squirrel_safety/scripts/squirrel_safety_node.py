#! /usr/bin/env python

import rospy

from squirrel_safety import safety


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_safety_node')
        safety_ = safety.SquirrelSafety()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
