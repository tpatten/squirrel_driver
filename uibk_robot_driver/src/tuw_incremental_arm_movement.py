#!/usr/bin/env python

import rospy
import readchar
import sys
from std_msgs.msg import Float64MultiArray, Int32
from sensor_msgs.msg import JointState
from robotino_msgs.srv import ResetOdometry

STEP = 0.02 # roughly 1 degree in radian
DOF = 8 # Number of Degrees of Freedom
joint_values = []

def publish(pub, joint_values):
    msg = Float64MultiArray()
    msg.layout.data_offset = 0
    msg.data = joint_values
    pub.publish(msg)

def callback(data):
    global joint_values
    joint_values = list(data.position)

def show_welcome():
    print("*"*80)
    print("*   Welcome to the simple node to incrementally move SQUIRREL's arm and base   *")
    show_help()

def show_help():
    print("*"*80)
    print("First select the axis to rotate \n\
           0: base platform x-axis\n\
           1: base platform y-axis\n\
           2: base platform rotation\n\
           3-7: axis counted from bottom to top")
    print("Second, move the axis with '+' or '-' keys.")
    print("Press 'h' for help.")
    print("Press 'r' to reset after a collision with the airskin.")
    print("Press 'q' to quit.")
    print("*"*80)

def reset_odometry():
    rospy.wait_for_service('reset_odometry')
    try:
        reset_odom = rospy.ServiceProxy('reset_odometry', ResetOdometry)
        resp1 = reset_odom(0.0, 0.0, 0.0)
        print "Service call to /reset_odometry was a success"
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    global joint_values, STEP, DOF
    rospy.init_node('incremental_arm_movement', anonymous=True)
    mode_pub = rospy.Publisher('/real/robotino/settings/switch_mode', Int32, queue_size=10, latch=True)
    pub = rospy.Publisher('/real/robotino/joint_control/move', Float64MultiArray, queue_size=10)
    rospy.Subscriber('/real/robotino/joint_control/get_state', JointState, callback)
    joint = None

    rospy.loginfo('incremental_arm_movement started')
    show_welcome() 
    mode_pub.publish(data=10)
  
    while True:
        try:
            c = readchar.readchar()
            if c.lower() == 'q':
                rospy.loginfo("Quit node")
                sys.exit()
            elif c.lower() == 'h':
                show_help()
            elif c.lower() == 'r':
                mode_pub.publish(data=10)
            elif c == '+' and joint is not None:
                rospy.loginfo("joint #{}: plus".format(joint))
                if len(joint_values) < DOF:
                    print('No data was received on /real/robotino/joint_control/get_state\n\
                           Unable to calculate a new position.\n\
                           Skipping movement')
                else:
                    tmp = joint_values[joint] + STEP
                    for i in xrange(len(joint_values)):
                        joint_values[i] = float('nan')
                    joint_values[joint] = tmp
                    publish(pub, joint_values)
                joint_values = []
            elif c == '-' and joint is not None:
                rospy.loginfo("joint #{}: minus".format(joint))
                if len(joint_values) < DOF:
                    print('No data was received on /real/robotino/joint_control/get_state\n\
                           Unable to calculate a new position.\n\
                           Skipping movement')
                else:
                    tmp = joint_values[joint] - STEP
                    for i in xrange(len(joint_values)):
                        joint_values[i] = float('nan')
                    joint_values[joint] = tmp
                    publish(pub, joint_values)
                joint_values = []
            elif abs(int(c)) <= DOF:
                rospy.loginfo(c)
                joint = int(c)
            else:
                rospy.loginfo("unknown")
        except ValueError as e:
            #print(e)
            print("Command not recognized or no joint selected.\nPress 'h' for instructions")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
