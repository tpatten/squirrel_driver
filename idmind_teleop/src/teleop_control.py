#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float64,Int64
from sensor_msgs.msg import Joy
from idmind_interaction.msg import MotorControl


class Joystick():
    def __init__(self):
        self.buttons = []
        self.axes = []
        self.word = ""
        self.flag = 0
        self.pub=rospy.Publisher('/teleop_buttons', String, queue_size=10)
        self.pub_head = rospy.Publisher("idmind_interaction/head",MotorControl,queue_size=10)
        self.pub_neck = rospy.Publisher("idmind_interaction/neck",MotorControl,queue_size=10)
        self.pub_headneck = rospy.Publisher("idmind_interaction/head_and_neck",MotorControl,queue_size=10)
        self.pub_camera = rospy.Publisher("idmind_interaction/camera_tilt",MotorControl,queue_size=10)
        self.msg = String()
        self.sub=rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_head_position = rospy.Subscriber("idmind_interaction/head_position",Int64,self.head_callback)
        self.sub_neck_position = rospy.Subscriber("idmind_interaction/neck_position",Int64,self.neck_callback)
        self.motor_msg = MotorControl()
        self.head_pos = 0
        self.neck_pos = 0

        self.motor_mode = 0
        self.lock_time = 0

    def joy_callback(self,msg):
        self.buttons = msg.buttons
        self.axes = msg.axes
        self.flag = 1

    def head_callback(self,msg):
        self.head_pos = msg.data
    def neck_callback(self,msg):
        self.neck_pos = msg.data


    def check_buttons(self):

        if self.buttons[4]:
            #LOOK LEFT
            if self.buttons[0]:
                self.play_button("left")
            #LOOK DOWN
            elif self.buttons[1]:
                self.play_button("down")
            #LOOK RIGHT
            elif self.buttons[2]:
                self.play_button("right")
            #CHEERFUL
            elif self.buttons[3]:
                self.play_button("cheerful")
        
        ###SIMPLE BLINKS###
        elif self.buttons[5]:    
            if self.buttons[0]:
                self.play_button("blue")
            elif self.buttons[1]:
                self.play_button("green")
            elif self.buttons[2]:
                self.play_button("red")
            elif self.buttons[3]:
                self.play_button("yellow")

        ### MOUTHS ###
        elif self.buttons[6]:
            if self.buttons[0]:
                self.play_button("mouth_blue")
            elif self.buttons[1]:
                self.play_button("mouth_green")
            elif self.buttons[2]:
                self.play_button("mouth_red")
            elif self.buttons[3]:
                self.play_button("mouth_yellow")


        #CONFUSED
        elif self.buttons[0]:
                self.play_button("confused")
        #GREETING
        elif self.buttons[1]:
            self.play_button("greeting")
        #NO
        elif self.buttons[2]:
            self.play_button("no")
        #THINK
        elif self.buttons[3]:
            self.play_button("think")    

        #######################################

        #DOOR#
        if self.buttons[9]:
            self.play_button("open")
        elif self.buttons[8]:
            self.play_button("close")


        #HEAD NECK MODE#
        if self.buttons[10] or self.buttons[11]:
            self.set_motor_mode()

        if self.axes[4] != 0:
            self.send_motors(self.axes[4])
        else:
            pass

        #CAMERA TILT
        if self.axes[5] != 0:
            self.move_camera(self.axes[5])


    def move_camera(self,move):
        if move > 0:
            self.motor_msg.position = 20
        else:
            self.motor_msg.position = 0
        self.pub_camera.publish(self.motor_msg)


    def set_motor_mode(self):
        if rospy.get_time() > self.lock_time + 0.3:
            enable = [self.buttons[10],self.buttons[11]]
            if enable == [1,1]:
                self.motor_mode = 2
                self.lock_time = rospy.get_time()
            elif enable == [0,1]:
                self.motor_mode = 1
            elif enable == [1,0]:
                self.motor_mode = 0


    def limit_motors(self):
        if self.motor_msg.position > 90:
           self.motor_msg.position = 90
        if self.motor_msg.position < -90:
           self.motor_msg.position = -90


    def send_motors(self,move):
        step = 10
        if move > 0:
            step = -step
        
        if self.motor_mode == 2:
            step = step / 1.25
            self.motor_msg.time = 2.5
            self.motor_msg.position = self.neck_pos + step
            self.limit_motors()
            self.pub_headneck.publish(self.motor_msg)
        elif self.motor_mode == 1:
            self.motor_msg.time = 3.0
            self.motor_msg.position = self.neck_pos + step
            self.limit_motors()
            self.pub_neck.publish(self.motor_msg)
        elif self.motor_mode == 0:
            self.motor_msg.time = 2.5
            self.motor_msg.position = self.head_pos + step  
            self.limit_motors()
            self.pub_head.publish(self.motor_msg)


    def play_button(self,message):
        self.msg.data = message
        self.pub.publish(self.msg)
        self.flag = 0

    def stop(self):
        pass


if __name__ == '__main__':
    rospy.init_node("teleop_idmind_control")
    jc=Joystick()
    rospy.on_shutdown(jc.stop)
    while not rospy.is_shutdown():
        if jc.flag == 1:
            jc.check_buttons()
        rospy.sleep(0.05)