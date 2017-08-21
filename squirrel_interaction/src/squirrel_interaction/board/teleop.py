#!/usr/bin/env python

""" The JoyController Node serves as an example client for the interaction components of SQUIRREL

    By subscribing to the /Joy [Joy] topic, this node allows the user to control:
        all motors of the interaction board.
        base leds,
        mouth leds,
        screen,
        speakers.

"""

import rospy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import degrees, radians


class JoyController:
    def __init__(self):
        rospy.init_node('squirrel_joy', anonymous=False)
        self._robot = RobotState()
        self.head_command = rospy.Publisher('/head_controller/joint_group_position_controller/command', Float64, queue_size=10)
        self.neck_command = rospy.Publisher('/neck_controller/joint_group_position_controller/command', Float64, queue_size=10)
        self.camera_command = rospy.Publisher('/camera_controller/joint_group_position_controller/command', Float64, queue_size=10)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.update_position)
        self.head = Head(self.head_command, self._robot)
        self.neck = Neck(self.neck_command, self._robot)
        self.camera = Camera(self.camera_command, self._robot)
        self._button_bindings = [
            self.look_forward,
            self.look_left,
            self.look_right,
            self.look_cheerful,
            self.head.move_left,
            self.head.move_right,
            do_nothing,
            do_nothing,
            do_nothing,
            do_nothing,
            do_nothing
        ]
        self._axes_bindings = [
            do_nothing,
            do_nothing,
            do_nothing,
            do_nothing,
            do_nothing,
            self.walk_x,
            self.neck.move,
            self.camera.move
        ]
        rospy.Subscriber('/joy', Joy, self.update_command)
        rospy.spin()

    def update_position(self, data):
        if data.name[0] == 'head' : self._robot.headPosition = int(degrees(data.position[0]))
        if data.name[0] == 'neck' : self._robot.neckPosition = int(degrees(data.position[0]))
        if data.name[0] == 'camera' : self._robot.cameraPosition = int(degrees(data.position[0]))

    def update_command(self, message):
        for index in range(0, len(message.buttons)):
            self._button_bindings[index](message.buttons[index])
        for index in range(0, len(message.axes)):
            self._axes_bindings[index](message.axes[index])

    def walk_x(self, distance):
        twist = Twist()
        twist.linear.x = 100 * distance
        self.cmd_vel.publish(twist)
        
    def look(self, image, true):
        if not true: return
        try:
            display_screen = rospy.ServiceProxy('face/emotion', DisplayScreen)
            display_screen(image)
        except rospy.ServiceException, e:
            rospy.logwarn('Failed to contact raspberry service.')

    def look_left(self, true):
        self.look('look_left', true)

    def look_right(self, true):
        self.look('look_right', true)

    def look_forward(self, true):
        self.look('look_front', true)

    def look_cheerful(self, true):
        self.look('cheerful', true)


def do_nothing(i):
    return


class Head:
    def __init__(self, pub, robot_state):
        self._l = 0
        self._r = 0
        self.pub = pub
        self._robot_state = robot_state

    def move_left(self, on):
        self._l = on

    # move_right is always called after move_left, so it can publish
    def move_right(self, on):
        self._r = on
        self._publish()

    def _publish(self):
        if self._l != self._r:
            self.pub.publish(radians(180) if self._r else radians(-180))
        else:
            self.pub.publish(radians(self._robot_state.headPosition))


class Neck(Head):
    def move(self, where):
        self.pub.publish(radians(self._robot_state.neckPosition) if not where else radians(where * -1 * 180))


class Camera(Head):
    """ Camera tilt is limited between -68 and 30 degrees """
    def move(self, where):
        if where == 0:
            destination = self._robot_state.cameraPosition
        else:
            destination = -68 if where < 0 else 30
        self.pub.publish(radians(destination))


class RobotState:
    def __init__(self):
        self.headPosition = 0
        self.neckPosition = 0
        self.cameraPosition = 0


if __name__ == '__main__':
    JoyController()




