#!/usr/bin/env python
# license removed for brevity

# read sensor data from Aruidno, and convert it into joint angle, 
# and pulish it to hand forward kinematics calculator

import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
import threading


class get_joint_angle(object):
    def __init__(self):

        self.callback_lock = threading.Lock();

        # 0 means sensor data is out of range, two possible reasons
        # (a)sensors may lose connections; (b) The motion is beyond the joint limitation
        # in either states, the robot should stop immediately  
        self.sensor_check_boolean = 1
        self.common_k = 9.0/28.0
        self.sensor_data = [0,0,0,0,0]
        self.joint_angle_value_data = JointState()
        self.hand_joint_angle = JointState()

        rospy.init_node('get_joint_angle_sensor', anonymous=True)

        self.palm_sensor1 = rospy.get_param("joint_sensor_calibration/palm_sensor1")
        self.palm_sensor2 = rospy.get_param("joint_sensor_calibration/palm_sensor2")
        self.finger_sensor3 = rospy.get_param("joint_sensor_calibration/finger_sensor3")
        self.finger_sensor4 = rospy.get_param("joint_sensor_calibration/finger_sensor4")
        self.finger_sensor5 = rospy.get_param("joint_sensor_calibration/finger_sensor5")

        self.left_finger_sensor_direction = rospy.get_param("left_finger_sensor_direction")

        # subscribe data from sensor
        rospy.Subscriber("joint_value_arduino", Int16MultiArray, self.call_back)
        joint_angle_value = rospy.Publisher('sensorToForwardKinematics', JointState, queue_size=10)
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            self.callback_lock.acquire();
            self.hand_angle_data = [self.map_palm_jointA(), self.map_palm_jointE(),
                                    self.map_left_finger_lower(), self.map_middle_finger_lower(), self.map_right_finger_lower()]
            self.callback_lock.release();

            #self.hand_angle_data = [self.joint_angle_value_data.position[0], self.joint_angle_value_data.position[4]]

           # if  self.sensor_data_limitation():
            self.hand_joint_angle.position = self.hand_angle_data
            joint_angle_value.publish(self.hand_joint_angle)

           # else:
                #stop the whole system
                #rospy.loginfo("there are some problems with hand joint sensors")
                #rospy.is_shutdown()

            rate.sleep()
        rospy.spin()


    def call_back(self,msg):

        self.callback_lock.acquire();
        self.joint_angle_value_data.position = msg.data
        for i in range(0, len(self.joint_angle_value_data.position)):
            self.sensor_data[i] = self.joint_angle_value_data.position[i]
        self.callback_lock.release();

# map the joint data to angle and pass data to forward kinematics solver

    def map_palm_jointA(self):
        self.palm_jointA_sensor = self.sensor_data[0]
        self.palm_jointA_0 = self.palm_sensor1
        self.palm_jointA_k = self.common_k
        self.palm_jointA_b = -self.common_k * self.palm_jointA_0
        self.palm_jointA_value = self.palm_jointA_k * self.palm_jointA_sensor + self.palm_jointA_b
        return self.palm_jointA_value

    def map_palm_jointE(self):
        self.palm_jointE_sensor = self.sensor_data[1]
        self.palm_jointE_0 = self.palm_sensor2
        self.palm_jointE_k = -self.common_k
        self.palm_jointE_b = -self.palm_jointE_k * self.palm_jointE_0
        self.palm_jointE_value = self.palm_jointE_k * self.palm_jointE_sensor + self.palm_jointE_b
        return self.palm_jointE_value

    def map_left_finger_lower(self):
        self.left_finger_lower_sensor = self.sensor_data[2]
        self.left_finger_lower_0 = self.finger_sensor3
        self.left_finger_lower_k = self.common_k * self.left_finger_sensor_direction
        self.left_finger_lower_b = -self.left_finger_lower_k * self.left_finger_lower_0
        self.left_finger_lower_value = self.left_finger_lower_k * self.left_finger_lower_sensor + self.left_finger_lower_b
        return self.left_finger_lower_value

    def map_middle_finger_lower(self):
        self.middle_finger_lower_sensor = self.sensor_data[3]
        self.middle_finger_lower_0 = self.finger_sensor4
        self.middle_finger_lower_k = -self.common_k
        self.middle_finger_lower_b = -self.middle_finger_lower_k * self.middle_finger_lower_0
        self.middle_finger_lower_value = self.middle_finger_lower_k * self.middle_finger_lower_sensor + self.middle_finger_lower_b
        return self.middle_finger_lower_value


    def map_right_finger_lower(self):
        self.right_finger_lower_sensor = self.sensor_data[4]
        self.right_finger_lower_0 = self.finger_sensor5
        self.right_finger_lower_k = -self.common_k
        self.right_finger_lower_b = -self.right_finger_lower_k * self.right_finger_lower_0
        self.right_finger_lower_value = self.right_finger_lower_k * self.right_finger_lower_sensor + self.right_finger_lower_b
        return self.right_finger_lower_value

    #for safety issue 

    def sensor_data_limitation(self):
        self.sensor_lower_boundary = [0, 0, -80.0, -80.0, -80.0]
        self.sensor_upper_boundary = [60.0, 90.0, 90.0, 90.0, 90.0]
        for i in [0,1,2,3,4]:
            if ((self.hand_angle_data[i] >= -90.0) & (self.hand_angle_data[i] <= 90.0)):
                self.sensor_check_boolean = True
                #print self.hand_angle_data[i]
            else:
                self.sensor_check_boolean = False
                break
                #rospy.is_shutdown()
        return self.sensor_check_boolean




if __name__ == '__main__':
    try:
        get_joint_angle()
    except rospy.ROSInterruptException: pass




'''

1. get joint angle value from arduino


'''

