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
        rate = rospy.Rate(100) # 10hz        
        self.impedance_signal_send = Float64MultiArray()

        self.torque_percs_received_sensor = Float64MultiArray()        
        rospy.Subscriber("torque_percs", Float64MultiArray, self.torque_percs_received)

        impedance_controller = rospy.Publisher('impedance_signal_send', Float64MultiArray, queue_size=10)

        self.torque_data_array = [0,0,0,0,0,0]

        self.left_finger_torque_data_array = [0 for i in range(10)]
        self.middle_finger_torque_data_array = [0 for i in range(10)]
        self.right_finger_torque_data_array = [0 for i in range(10)]

        self.aver_force_left = 0
        self.aver_force_middle = 0
        self.aver_force_right = 0

        self.left_value_set_to_zero = 0
        self.middle_value_set_to_zero = 0
        self.right_value_set_to_zero = 0
        
        self.left_value_sum = 0
        self.middle_value_sum = 0
        self.right_value_sum = 0
        
        self.value_count = 0

    ### Parameters of the desired dynamics
        # k * (T_e*dot_e + e) = f_d - f
        self.T_e = 0.3 # desired time constant
        self.k = 20    # desired spring
        
        self.F_ext = 1
        self.T = 0

        self.error_n_3 = 0 # left finger
        self.error_n_4 = 0 # middle finger
        self.error_n_5 = 0 # right finger
        
        self.proxy_n_3 = 0
        self.proxy_n_4 = 0
        self.proxy_n_5 = 0
        

        self.scalar = 20.0

        while not rospy.is_shutdown():
            # update the impedance signal by the force sensor 
            
            self.error_n_3 = self.output(self.error_n_3, self.torque_data_array[1], 15, 0.02) # 20 is ok
            self.error_n_4 = self.output(self.error_n_4, self.torque_data_array[0], 0, 0.02) # 20 is ok
            self.error_n_5 = self.output(self.error_n_5, self.torque_data_array[2], 15, 0.02) # 20 is ok  
            
            self.proxy_n_3 = self.torque_data_array[4]
            self.proxy_n_4 = self.torque_data_array[3]
            self.proxy_n_5 = self.torque_data_array[5]
            
            

            self.impedance_signal_data = [0, 0, self.error_n_3 * self.scalar, self.error_n_4 * 20, self.error_n_5 * self.scalar, self.proxy_n_3, self.proxy_n_4, self.proxy_n_5] 
            #self.impedance_signal_data = [0, 0, self.torque_data_array[1], self.torque_data_array[0], self.torque_data_array[2], self.proxy_n_3, self.proxy_n_4, self.proxy_n_5] 
                        
            #self.impedance_signal_data = [0, 0, self.aver_force_left, self.aver_force_middle, self.aver_force_right] 
            
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
            self.torque_data_array[i] = self.torque_percs_received_sensor.data[i]
        
        #if (abs(self.aver_force_left-self.torque_data_array[1]) > 2):
        if (self.value_count < 20):
            self.left_value_sum += self.torque_data_array[1]
            self.middle_value_sum += self.torque_data_array[0]
            self.left_value_sum += self.torque_data_array[2]
        if (self.value_count == 20):
            self.left_value_set_to_zero = self.left_value_sum / 20
            self.middle_value_set_to_zero = self.middle_value_sum / 20
            self.right_value_set_to_zero = self.right_value_sum / 20
        if (self.value_count > 20):
        #Signal filter

            self.torque_data_array[1] = self.torque_data_array[1]- self.left_value_set_to_zero 
            self.torque_data_array[0] = self.torque_data_array[0]- self.middle_value_set_to_zero 
            self.torque_data_array[2] = self.torque_data_array[2]- self.right_value_set_to_zero 

            #self.aver_force_left = self.aver_list_value(self.left_finger_torque_data_array, self.torque_data_array[1], self.aver_force_left)       
            #self.aver_force_middle = self.aver_list_value(self.middle_finger_torque_data_array, self.torque_data_array[0], self.aver_force_middle)
            #self.aver_force_right = self.aver_list_value(self.right_finger_torque_data_array, self.torque_data_array[2], self.aver_force_right)
        
        self.value_count = self.value_count + 1
        

    def aver_list_value(self, list_value, new_value, old_value):
        sum_value = 0
        if (abs(new_value - old_value) > 3):
            list_value.append(new_value)
            list_value.pop(1)
        for i in range(len(list_value)):
            sum_value = sum_value + list_value[i]
        return sum_value / len(list_value)




if __name__ == '__main__':
    try:
        AdmittanceControl()
    except rospy.ROSInterruptException: pass

