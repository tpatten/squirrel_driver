#!/usr/bin/env python

import rospy
import sys
from idmind_interaction.msg import MotorControl


class Motors():
    def __init__(self):
        self.pos = None
        self.t = None 
        self.device = 0
        self.pub_head = rospy.Publisher("idmind_interaction/head",MotorControl,queue_size=10)
        self.pub_neck = rospy.Publisher("idmind_interaction/neck",MotorControl,queue_size=10)
        self.pub_headneck = rospy.Publisher("idmind_interaction/head_and_neck",MotorControl,queue_size=10)
        rospy.sleep(0.4) #wait for publishers to be ready

    def sendMotors(self):
        msg = MotorControl()
        msg.position = self.pos
        msg.time = self.t
        if self.device == 2:
            self.pub_headneck.publish(msg)
        elif self.device == 0:
            self.pub_head.publish(msg)
        elif self.device == 1:
            self.pub_neck.publish(msg)


if __name__ == "__main__":
    
    rospy.init_node("squirrel_motors_api")
    m = Motors()
    
    if len(sys.argv)==3:
        m.device = int(sys.argv[1])
        m.t = float(sys.argv[2])
        while not rospy.is_shutdown():
            pos_str = raw_input("position: ")
            m.pos = int(pos_str)
            m.sendMotors()
    elif len(sys.argv)==4:
        m.device = int(sys.argv[1])
        m.t = float(sys.argv[2])
        m.pos = int (sys.argv[3])
        m.sendMotors()
    elif len(sys.argv)==1:
        m.device=2
        m.t = 4
        i = 0
        while not rospy.is_shutdown():
            if i % 2 == 0:
                m.pos=180
            elif i%2 == 1:
                m.pos=0
            i+=1    
            enter = raw_input("Press Enter to continue")
            m.sendMotors()
            