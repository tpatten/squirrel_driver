import rospy

from squirrel_safety_msgs.msg import Safety
from std_msgs.msg import Bool

from squirrel_manipulation_msgs.srv import SoftHandGrasp
from geometry_msgs.msg import Pose, PoseStamped
from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback
from visualization_msgs.msg import Marker

class SquirrelSafety(object):
    def __init__(self):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')

        self.wrist_sub = rospy.Subscriber('/wrist/wrist_bumper', Bool, self.wrist_callback)
        self.bumper_sub = rospy.Subscriber('/bumper', Bool, self.bumper_callback)
        self.airskin_sub = rospy.Subscriber('arm_bumper', Bool, self.airskin_callback)
        self.safety_pub = rospy.Publisher('/squirrel_safety', Safety, queue_size=10)


    def airskin_callback(self, msg):
      if msg:
        safety_msg = Safety()
        safety_msg.emergency_state = True
        safety_msg.airskin_stop = True
        self.safety_pub.publish(safety_msg)


    def wrist_callback(self, msg):
      if msg:
        safety_msg = Safety()
        safety_msg.emergency_state = True
        safety_msg.wrist_stop = True
        self.safety_pub.publish(safety_msg)


    def bumper_callback(self, msg):
      if msg:
        safety_msg = Safety()
        safety_msg.emergency_state = True
        safety_msg.bumper_stop = True
        self.safety_pub.publish(safety_msg)


