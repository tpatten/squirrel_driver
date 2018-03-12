#!/usr/bin/env python

"""
    This Ros Node wraps the serial_api to communicate with the Interaction Board

    The name of the port to communicate with is retrieved from the "/squirrel_port" rosparam
    It defaults to /dev/ttyUSB0

    At a rate of 20Hz, it publishes motor positions to relevant topics, and actuates all devices

    For most motors it is possible to specify an absolute or relative angular speed, in radians

    Clients may also change motor velocities within certain limits. 
    Limits are as follows:
    2 <= head_pan <= 30
    2 <= neck_pan <= 30
    2 <= neck_tilt <= 25
    door = 20

    List of topics:
        OUT
            /joint_states [JointState]
        IN
            head_controller/command
            head_controller/speed
            neck_pan_controller/command
            neck_pan_controller/speed
            neck_tilt_controller/command
            neck_tilt_controller/speed
            head_and_neck_controller/command
            /light/command
            /light_complex/command
            /mouth/command

    The door is controlled via a Service.

            door_controller/command {'open', 'close' }

"""

import rospy
import serial_api
from serial import SerialException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, ColorRGBA, String, Header, Float64MultiArray, Int16, UInt16MultiArray
from squirrel_interaction.srv import DoorController
from math import degrees, radians


class Controller:
    """Ros node to interfaces with interaction board """
    def __init__(self):
        try:
            rospy.init_node('squirrel_driver', anonymous=False)
            self._open_devices()
            self.head_position_reference = 0
            self.neck_pan_position_reference = 0
            self.neck_tilt_position_reference = 0
            self.head_speed_reference = None
            self.neck_pan_speed_reference = None
            self.neck_tilt_speed_reference = None
            self.base_lights = None
            self.mouth_lights = None
            self.door = 'OPENING'
            self.to_close_door = False
            self.to_open_door = True
        except SerialException:
            rospy.logwarn('Failed to open some devices')
        rospy.loginfo('Started squirrel_driver node')
        rospy.Subscriber('head_controller/command', Float64, self.move_head)
        rospy.Subscriber('neck_pan_controller/command', Float64, self.move_neck_pan)
        rospy.Subscriber('neck_pan_controller/rel_command', Float64, self.move_neck_pan_rel)
        rospy.Subscriber('neck_tilt_controller/command', Float64, self.move_neck_tilt)
        rospy.Subscriber('neck_tilt_controller/rel_command', Float64, self.move_neck_tilt_rel)
        rospy.Subscriber('head_and_neck_controller/command', Float64MultiArray, self.move_head_and_neck)
        rospy.Subscriber('head_controller/speed', Int16, self.set_head_speed)
        rospy.Subscriber('neck_pan_controller/speed', Int16, self.set_neck_pan_speed)
        rospy.Subscriber('neck_tilt_controller/speed', Int16, self.set_neck_tilt_speed)

        rospy.Subscriber('/light/command', ColorRGBA, self.change_base_light)
        rospy.Subscriber('/light_complex/command', UInt16MultiArray, self.change_base_light_complex)
        rospy.Subscriber('/mouth/command', ColorRGBA, self.change_mouth_light)
        rospy.Service('door_controller/command', DoorController, self.move_door)
        self.position_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.door_state_pub = rospy.Publisher('/door_state', String, queue_size=10)

    def run(self):
        rate = rospy.Rate(20)
        last_correct_positions = [0.0, 0.0, 0.0, 0.0]
        while not rospy.is_shutdown():
            #  READ
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['head_joint', 'neck_pan_joint', 'neck_tilt_joint', 'door_joint']
            positions = self._motor.get_all_positions()
            if positions:
                joint_state_msg.position = map(radians, positions)
                last_correct_positions = joint_state_msg.position
            else:
                joint_state_msg.position = last_correct_positions
                joint_state_msg.velocity = []
                joint_state_msg.effort = []
            self.position_pub.publish(joint_state_msg)

            door = self._motor.get_door_status()
            #if door in ['OPEN', 'CLOSED', 'AJAR']:
            #    self.door = door
            #    self.door_state_pub.publish(door)
            if door == 'OPEN' and self.door == 'OPENING':
                self.door = 'OPEN'
            if door == 'CLOSED' and self.door == 'CLOSING':
                self.door = 'CLOSED'
            self.door_state_pub.publish(self.door)
            #  WRITE
            #  speeds
            
            if self.head_speed_reference is not None:
                if self._motor.set_motor_speed('head', self.head_speed_reference):
                    self.head_speed_reference = None
            if self.neck_pan_speed_reference is not None:
                if self._motor.set_motor_speed('neck', self.neck_pan_speed_reference):
                    self.neck_pan_speed_reference = None
            if self.neck_tilt_speed_reference is not None:
                if self._motor.set_motor_speed('camera', self.neck_tilt_speed_reference):
                    self.neck_tilt_speed_reference = None

            #  other motors
            #########################################################################
            #self._motor.move_all(
            #    self.head_position_reference,
            #    self.neck_pan_position_reference,
            #    self.neck_tilt_position_reference
            #)
            #########################################################################
            if self.head_position_reference is not None:
                if self._motor.move_to('head', self.head_position_reference):
                    self.head_position_reference = None
            if self.neck_pan_position_reference is not None:
                if self._motor.move_to('neck', self.neck_pan_position_reference):
                    self.neck_pan_position_reference = None
            if self.neck_tilt_position_reference is not None:
                if self._motor.move_to('camera', self.neck_tilt_position_reference):
                    self.neck_tilt_position_reference = None

            #  door
            if self.to_close_door:
                self.door = 'CLOSING'
                if self._motor.move_to('door', 30000):
                    self.to_close_door = False

            if self.to_open_door:
                self.door = 'OPENING'
                if self._motor.move_to('door', -30000):
                    self.to_open_door = False
            # lights
            if self.mouth_lights:
                if self._motor.set_mouth_led_colors(self.mouth_lights):
                    self.mouth_lights = None
            
            if self.base_lights:
                if self._motor.set_base_led_colors(self.base_lights):
                    self.base_lights = None

            rate.sleep()

    def _open_devices(self):
        port = rospy.get_param("/squirrel_port") if rospy.has_param("/squirrel_port") else "/dev/ttyBoard"
        self._motor = serial_api.Controller(port, 115200, timeout=0.05)

    def set_head_speed(self, message):
        speed = message.data
        if not self._motor.set_motor_speed('head', speed): 
            self.head_speed_reference = speed
        else:
            self.head_speed_reference = None

    def set_neck_pan_speed(self, message):
        speed = message.data
        if not self._motor.set_motor_speed('neck', speed):
            self.neck_pan_speed_reference = speed
        else:
            self.neck_pan_speed_reference = None

    def set_neck_tilt_speed(self, message):
        speed = message.data
        if not self._motor.set_motor_speed('camera', speed):
            self.neck_tilt_speed_reference = message.data
        else:
            self.neck_tilt_speed_reference = None

    def move_head(self, message):
        reference = int(degrees(message.data))
        if not self._motor.move_to('head', reference):
            self.head_position_reference = int(degrees(message.data))
        else:
            self.head_position_reference = None

    def move_neck_pan(self, message):
        reference = int(degrees(message.data))
        if not self._motor.move_to('neck', reference):
            self.neck_pan_position_reference = int(degrees(message.data))
        else:
            self.neck_pan_position_reference = None

    def move_neck_pan_rel(self, message):
        position = self._motor.get_position("neck")
        reference = position + int(degrees(message.data))
        if not self._motor.move_to('neck', reference):
            self.neck_pan_position_reference = reference
        else:
            self.neck_pan_position_reference = None

    def move_neck_tilt(self, message):
        reference = int(degrees(message.data))
        if not self._motor.move_to('camera', reference):
            self.neck_tilt_position_reference = reference
        else:
            self.neck_tilt_position_reference = None

    def move_neck_tilt_rel(self, message):
        position = self._motor.get_position("camera")
        reference = int(degrees(message.data))
        if not self._motor.move_to('camera', reference):
            self.neck_tilt_position_reference = reference
        else:
            self.neck_tilt_position_reference = None

    def move_head_and_neck(self, message):
        """ moves head_pan, neck_pan and neck_tilt to message.data[0, 1, 2] """
        head_reference = int(degrees(message.data[0]))
        neck_pan_reference = int(degrees(message.data[1]))
        neck_tilt_reference = int(degrees(message.data[2]))
        if not self._motor.move_all(head_reference, neck_pan_reference, neck_tilt_reference):
            self.head_position_reference = head_reference
            self.neck_pan_position_reference = neck_pan_reference
            self.neck_tilt_position_reference = neck_tilt_reference
        else:
            self.head_position_reference = None
            self.neck_pan_position_reference = None
            self.neck_tilt_position_reference = None

    def change_base_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)] * 42
        if not self._motor.set_base_led_colors(color):
            self.base_lights = color
        else:
            self.base_lights = None

    def change_base_light_complex(self, message):
        colors = message.data
        if len(colors) != 126:
            rospy.logwarn('failed to send complex light command. need RGB component for each led (42 * 3 = 126')
        self.base_lights = colors
        #else:
        #    if not self._motor.set_base_led_colors(colors):
        #        self.base_lights = colors
        #    else:
        #        self.base_lights = None

    def change_mouth_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)] * 4
        if not self._motor.set_mouth_led_colors(color):
            self.mouth_lights = color   # number of base leds
        else:
            self.mouth_lights = None

    def move_door(self, req):
        if req.message == 'open':
            self.to_open_door = True
            return True
        elif req.message == 'close':
            self.to_close_door = True
            return True
        return False


if __name__ == '__main__':
    controller = Controller()
    controller.run()
