#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16, Float64MultiArray, ColorRGBA

colors = {
	'red': ColorRGBA(255, 0, 0, 0),
	'green': ColorRGBA(0, 255, 0, 0),
	'blue': ColorRGBA(0, 0, 255, 0)
}

class Test:
	def __init__(self):
		rospy.init_node("test", anonymous=True)
		self.reference = rospy.Publisher("/head_and_neck_controller/command", Float64MultiArray, queue_size=10)
		self.headspeed = rospy.Publisher("/head_controller/speed", Int16, queue_size=10)
		self.neckpanspeed = rospy.Publisher("/neck_pan_controller/speed", Int16, queue_size=10)
		self.necktiltspeed = rospy.Publisher("/neck_tilt_controller/speed", Int16, queue_size=10)
		self.mouth_pub = rospy.Publisher("mouth/command", ColorRGBA, queue_size=10)
		self.base_pub = rospy.Publisher("light/command", ColorRGBA, queue_size=10)

	def set_speeds(self, head, neckp, neckt):
     		self.headspeed.publish(head)
     		self.neckpanspeed.publish(neckp)
		self.necktiltspeed.publish(neckt)
 
	def move(self, head, neckp, neckt):
     		msg = Float64MultiArray()
    		msg.data = [head, neckp, neckt]
     		self.reference.publish(msg)

	def mouth(self, color):
		self.mouth_pub.publish(colors[color])

	def light(self, color):
		self.base_pub.publish(colors[color])


if __name__ == '__main__':
	Test()
	rospy.spin()
