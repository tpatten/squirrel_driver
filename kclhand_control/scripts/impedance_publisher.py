#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import copy
import numpy as np
import math as m

                
class AdmittanceControl(object):

    def __init__(self):
        rospy.init_node('hand_impedance_control', anonymous=True)
        rate = rospy.Rate(50) # 10hz        
        self.impedance_signal_send = Float64MultiArray()

        self.torque_percs_received_sensor = Float64MultiArray()        
        rospy.Subscriber("torque_percs", Float64MultiArray, self.torque_percs_received)

        impedance_controller = rospy.Publisher('impedance_signal_send', Float64MultiArray, queue_size=10)

        self.torque_data_array = [0,0,0,0,0,0]

    ### Parameters of the desired dynamics
        # k * (T_e*dot_e + e) = f_d - f
        self.T_e = 0.3 # desired time constant
        self.k = 20    # desired spring
        
        self.F_ext = 1
        self.T = 0

        self.error_n_3 = 0 # left finger
        self.error_n_4 = 0 # middle finger
        self.error_n_5 = 0 # right finger

        self.scalar = 20.0

        while not rospy.is_shutdown():
            # update the impedance signal by the force sensor 
            self.error_n_3 = self.output(self.error_n_3, self.torque_data_array[1], 0, 0.02) # 20 is ok
            self.error_n_4 = self.output(self.error_n_4, self.torque_data_array[0], 0, 0.02) # 20 is ok
            self.error_n_5 = self.output(self.error_n_5, self.torque_data_array[2], 0, 0.02) # 20 is ok  
            

            self.impedance_signal_data = [0, 0, self.error_n_3 * self.scalar, self.error_n_4 * self.scalar, self.error_n_5 * self.scalar] 
            self.impedance_signal_send.data = self.impedance_signal_data
            impedance_controller.publish(self.impedance_signal_send)
            rate.sleep()

        rospy.spin()

    
    def output(self, e_0, F_ext, F_d, T):

        # input parameters
        self.e_0 = e_0
        self.F_ext = F_ext
        self.F_d = F_d
        self.T = T
        
        # calculation of the trajectory position in the next step
        self.e_1 = (1 - self.T/self.T_e)*self.e_0 + self.T/(self.k*self.T_e)*\
                   (self.F_d - self.F_ext)
        
        return self.e_1
    

    def torque_percs_received(self,msg):
        self.torque_percs_received_sensor.data = msg.data
   
        for i in range(0, len(self.torque_percs_received_sensor.data)):
            self.torque_data_array[i] = self.torque_percs_received_sensor.data[i] * 1.0


if __name__ == '__main__':
    try:
        AdmittanceControl()
    except rospy.ROSInterruptException: pass

