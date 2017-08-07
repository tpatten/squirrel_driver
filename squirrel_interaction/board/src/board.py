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
            /head_controller/joint_group_position_controller/command
            /neck_controller/joint_group_position_controller/command
            /camera_controller/joint_group_position_controller/command
            /light/command
"""

import rospy
import serial_api
from serial import SerialException

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, ColorRGBA

from math import degrees, radians


def _valid(data):
    return data is not None


def _valid_camera(data):
    return -1.2 < data < 0.53


class Controller:
    """Ros node to interfaces with interaction board """
    def __init__(self):
        try:
            self._open_devices()
            self.head_destination = self._motor.get_position("head")
            self.neck_destination = self._motor.get_position("neck")
            self.camera_destination = self._motor.get_position("camera")
            self.base_lights = [0, 0, 0] * 84 # turns off all base leds
        except SerialException:
            rospy.logwarn('Failed to open some devices')
        rospy.init_node('squirrel_driver', anonymous=False)
        print 'Started squirrel_driver node'
	rospy.Subscriber('/head_controller/joint_group_position_controller/command', Float64, self.move_head)
        rospy.Subscriber('/neck_controller/joint_group_position_controller/command', Float64, self.move_neck)
        rospy.Subscriber('/camera_controller/joint_group_position_controller/command', Float64, self.move_camera)
        rospy.Subscriber('/light/command', ColorRGBA, self.change_base_light)
        self.position_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def run(self):
        self.rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._motor.move_to("head", self.head_destination)
            self._motor.move_to("neck", self.neck_destination)
            self._motor.move_to("camera", self.camera_destination)
            self._motor.set_base_led_colors(self.base_lights)
            position = JointState(
                name=["head"],
                position=[radians(self._motor.get_position("head"))])
            self.position_pub.publish(position)
            position = JointState(
                name=["neck"],
                position=[radians(self._motor.get_position("neck"))])
            self.position_pub.publish(position)
            position = JointState(
                name=["camera"],
                position=[radians(self._motor.get_position("camera"))])
            self.position_pub.publish(position)

            self.rate.sleep()

    def _open_devices(self):
        port = rospy.get_param("/squirrel_port") if rospy.has_param("/squirrel_port") else "/dev/ttyUSB0"
        self._motor = serial_api.Controller(port, 115200)

    def move_head(self, message):
        if _valid(message) and self._motor is not None:
            self.head_destination = int(degrees(message.data))

    def move_neck(self, message):
        if _valid(message) and self._motor is not None:
            self.neck_destination = int(degrees(message.data))

    def move_camera(self, message):
        if _valid_camera(message) and self._motor is not None:
            self.camera_destination = int(degrees(message.data))

    def change_base_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)]
        lights = []
        for x in range(84):
            lights.extend(color)
        self.base_lights = lights


if __name__ == '__main__':
    controller = Controller()
    controller.run()



