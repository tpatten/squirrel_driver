#!/usr/bin/env python

"""
    This Ros Node wraps the serial_api to communicate with the Interaction Board

    The name of the port to communicate with is retrieved from the "/squirrel_port" rosparam
    It defaults to /dev/ttyUSB0

    At a rate of 20Hz, it publishes motor positions to relevant topics, and actuates all devices

    Clients may also change motor velocities within certain limits. 
    Limits are as follows:
	10 <= head_pan <= 30
	10 <= neck_pan <= 30
	10 <= neck_tilt <= 30
	door = 20
	
    List of topics:
        OUT
            /joint_states [JointState]
        IN
            head_controller/command
            neck_pan_controller/command
            neck_tilt_controller/command
	    head_and_neck_controller/command
            /light/command
	    /mouth/command
"""

import rospy
import serial_api
from serial import SerialException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, ColorRGBA, String, Header, Float64MultiArray, Int16
from squirrel_interaction.srv import DoorController
from math import degrees, radians


class Controller:
    """Ros node to interfaces with interaction board """
    def __init__(self):
        try:
            rospy.init_node('squirrel_driver', anonymous=False)
            self._open_devices()
            self.head_destination = self._motor.get_position("head")
            self.neck_destination = self._motor.get_position("neck")
            self.camera_destination = self._motor.get_position("camera")
            self.base_lights = [0, 0, 0] * 42 # turns off all base leds
	    self.mouth_lights = [0, 0, 0] * 4 # turns off all mouth leds
            self.door_open = False
            self.door_closed = False
            #self.should_open_door = False
            #self.should_close_door = False
        except SerialException:
            rospy.logwarn('Failed to open some devices')
        rospy.loginfo('Started squirrel_driver node')
        rospy.Subscriber('head_controller/command', Float64, self.move_head)
        rospy.Subscriber('neck_pan_controller/command', Float64, self.move_neck)
        rospy.Subscriber('neck_pan_controller/rel_command', Float64, self.move_neck_rel)
        rospy.Subscriber('neck_tilt_controller/command', Float64, self.move_camera)
        rospy.Subscriber('neck_tilt_controller/rel_command', Float64, self.move_camera_rel)
        rospy.Subscriber('head_and_neck_controller/command', Float64MultiArray, self.move_head_and_neck)
	rospy.Subscriber('head_controller/speed', Int16, self.set_head_speed)
	rospy.Subscriber('neck_pan_controller/speed', Int16, self.set_neck_pan_speed)
	rospy.Subscriber('neck_tilt_controller/speed', Int16, self.set_neck_tilt_speed)
	 
	rospy.Subscriber('/light/command', ColorRGBA, self.change_base_light)
	rospy.Subscriber('/mouth/command', ColorRGBA, self.change_mouth_light)
        rospy.Service('door_controller/command', DoorController, self.move_door)
        self.position_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def run(self):
        rate = rospy.Rate(20)
	last_correct_positions = [0.0, 0.0, 0.0, 0.0]
        while not rospy.is_shutdown():
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['head_joint', 'neck_pan_joint', 'neck_tilt_joint', 'door_joint']
	    positions = self._motor.get_all_positions()
	    if positions:
            	joint_state_msg.position = map(radians, positions)
		last_correct_positions = joint_state_msg.position
	    else:
		rospy.loginfo('incorrect motor position reads. using last known correct')
		joint_state_msg.position = last_correct_positions
	    joint_state_msg.velocity = []
      	    joint_state_msg.effort = []
            self.position_pub.publish(joint_state_msg)
            rate.sleep()

    def _open_devices(self):
        port = rospy.get_param("/squirrel_port") if rospy.has_param("/squirrel_port") else "/dev/ttyBoard"
        self._motor = serial_api.Controller(port, 115200, timeout=0.02)
	
    def set_head_speed(self, message):
	self._motor.set_motor_speed("head", message.data)

    def set_neck_pan_speed(self, message):
        self._motor.set_motor_speed("neck", message.data)

    def set_neck_tilt_speed(self, message):
        self._motor.set_motor_speed("camera", message.data)

    def move_head(self, message):
        self.head_destination = int(degrees(message.data))
        self._motor.move_to("head", self.head_destination)

    def move_neck(self, message):
        rospy.loginfo("received {} on move_neck".format(message.data))
        self.neck_destination = int(degrees(message.data))
        self._motor.move_to("neck", self.neck_destination)

    def move_neck_rel(self, message):
        position = self._motor.get_position("neck")
        print("current neck position: {}".format(position))
        self.neck_destination = position + int(degrees(message.data))
        print("next neck position: {}".format(self.neck_destination))
        self._motor.move_to("neck", self.neck_destination)

    def move_camera(self, message):
        self.camera_destination = int(degrees(message.data))
        self._motor.move_to("camera", self.camera_destination)

    def move_camera_rel(self, message):
        position = self._motor.get_position("camera")
        print("current camera position: {}".format(position))
        self.camera_destination = position + int(degrees(message.data))
        print("next camera position: {}".format(self.camera_destination))
        self._motor.move_to("camera", self.camera_destination)

    def move_head_and_neck(self, message):
	""" moves head_pan, neck_pan and neck_tilt to message.data[0, 1, 2] """
        self.head_destination = int(degrees(message.data[0]))
	self.neck_destination = int(degrees(message.data[1]))
	self.camera_destination = int(degrees(message.data[2]))
	self._motor.move_all(self.head_destination, self.neck_destination, self.camera_destination)


    def change_base_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)]
        self.base_lights = color * 42   # number of base leds
        self._motor.set_base_led_colors(self.base_lights)

    def change_mouth_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)]
        self.mouth_lights = color * 4   # number of base leds
        self._motor.set_mouth_led_colors(self.mouth_lights)

    def move_door(self, req):
        if req.message == 'open':
            self._motor.move_to('door', -20000)
            while not self.door_open:
                pass
            return True
        elif req.message == 'close':
            self._motor.move_to('door', 30000)
            while not self.door_closed:
                pass
            return True
        return False

if __name__ == '__main__':
    controller = Controller()
    controller.run()



