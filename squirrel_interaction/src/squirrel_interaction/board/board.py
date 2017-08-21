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
from std_msgs.msg import Float64, ColorRGBA, String
from squirrel_interaction.srv import DoorController
from math import degrees, radians


class Controller:
    """Ros node to interfaces with interaction board """
    def __init__(self):
        try:
            self._open_devices()
            self.head_destination = self._motor.get_position("head")
            self.neck_destination = self._motor.get_position("neck")
            self.camera_destination = self._motor.get_position("camera")
            self.base_lights = [0, 0, 0] * 84 # turns off all base leds
            self.door_open = False
            self.door_closed = False
            self.should_open_door = False
            self.should_close_door = False
        except SerialException:
            rospy.logwarn('Failed to open some devices')
        rospy.init_node('squirrel_driver', anonymous=False)
        print 'Started squirrel_driver node'
        rospy.Subscriber('/head_controller/joint_group_position_controller/command', Float64, self.move_head)
        rospy.Subscriber('/neck_controller/joint_group_position_controller/command', Float64, self.move_neck)
        rospy.Subscriber('/camera_controller/joint_group_position_controller/command', Float64, self.move_camera)
        rospy.Subscriber('/light/command', ColorRGBA, self.change_base_light)
        rospy.Service('/door_controller/command', DoorController, self.move_door)
        self.position_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._motor.move_to("head", self.head_destination)
            self._motor.move_to("neck", self.neck_destination)
            self._motor.move_to("camera", self.camera_destination)
            self._motor.set_base_led_colors(self.base_lights)
            door = self._motor.get_door_status()
            self.door_open = door == 'OPEN'
            self.door_closed = door == 'CLOSED'
            if self.should_open_door:
                self._motor.move_to('door', -20000)
            if self.should_close_door:
                self._motor.move_to('door', 30000)
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

            rate.sleep()

    def _open_devices(self):
        port = rospy.get_param("/squirrel_port") if rospy.has_param("/squirrel_port") else "/dev/ttyUSB0"
        self._motor = serial_api.Controller(port, 115200)

    def move_head(self, message):
        self.head_destination = int(degrees(message.data))

    def move_neck(self, message):
        self.neck_destination = int(degrees(message.data))

    def move_camera(self, message):
        self.camera_destination = int(degrees(message.data))

    def change_base_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)]
        self.base_lights = color * 42   # number of base leds

    def move_door(self, req):
        if req.message == 'open':
            self.should_open_door = True
            self.should_close_door = False
            while not self.door_open:
                pass
            return True
        elif req.message == 'close':
            self.should_open_door = False
            self.should_close_door = True
            while not self.door_closed:
                pass
            return True
        return False

if __name__ == '__main__':
    controller = Controller()
    controller.run()



