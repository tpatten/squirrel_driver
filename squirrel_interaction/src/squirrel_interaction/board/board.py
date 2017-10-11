#!/usr/bin/env python

"""
    This Ros Node wraps the serial_api to communicate with the Interaction Board

    The name of the port to communicate with is retrieved from the "/squirrel_port" rosparam
    It defaults to /dev/ttyUSB0

    At a rate of 50Hz, it publishes motor positions to relevant topics, and actuates all devices

    List of topics:
        OUT
            /joint_states [JointState]
        IN
            head_controller/command
            neck_pan_controller/command
            neck_tilt_controller/command
            /light/command
	    /mouth/command
"""

import rospy
import serial_api
from serial import SerialException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, ColorRGBA, String, Header
from squirrel_interaction.srv import DoorController
from squirrel_interaction.msg import BaseLights
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
        rospy.Subscriber('/light/command', ColorRGBA, self.change_base_light)
	rospy.Subscriber('/light_complex/command', BaseLights, self.change_base_lights_complex) 
	rospy.Subscriber('/mouth/command', ColorRGBA, self.change_mouth_light)
        rospy.Service('door_controller/command', DoorController, self.move_door)
        self.position_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['head_joint', 'neck_pan_joint', 'neck_tilt_joint', 'door_joint']
            joint_state_msg.position = [radians(self._motor.get_position("head")), radians(self._motor.get_position("neck")), radians(self._motor.get_position("camera")), radians(self._motor.get_position("door"))]
            joint_state_msg.velocity = []
            joint_state_msg.effort = []
            self.position_pub.publish(joint_state_msg)
            rate.sleep()

    def _open_devices(self):
        port = rospy.get_param("/squirrel_port") if rospy.has_param("/squirrel_port") else "/dev/ttyBoard"
        self._motor = serial_api.Controller(port, 115200)

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

    def change_base_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)]
        self.base_lights = color * 42   # number of base leds
        self._motor.set_base_led_colors(self.base_lights)

    def change_base_light_complex(self, message):
        if len(message.colors) is not 126:
	    rospy.log_warn('for individual LED colors, specify an RGB value for each of the 42 LEDS')
	    return
	self.base_lights = message.colors
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



