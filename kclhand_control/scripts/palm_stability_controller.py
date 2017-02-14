#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import hand_controller_palm_xz as palm_stability
import copy

class palm_stability_calculator(object):
    def __init__(self):
        # subscribe the palm joint displancement, velocitys, out put the control force 
        rospy.init_node('palm_control_signal_calculator', anonymous=True)
        self.palm_controller_data_send = Float64MultiArray()
        #palm_controller_data_send = Float64MultiArray()     
        self.data_received =  JointState()
        rospy.Subscriber("sensorToForwardKinematics", JointState, self.palm_displacement_velocity_call_back)
       
        palm_force_controller = rospy.Publisher('palm_control_signal', Float64MultiArray, queue_size=10)

        self.data_array = [3.14, 0, 0, 0, 0]
        self.palm_force_calculator = palm_stability.Controller()
        rate = rospy.Rate(300) # 10hz
        self.theta0 = 3.14
        self.theta1 = 0

        self.time_begin = rospy.get_rostime()
        self.time_end = rospy.get_rostime()
        self.duration_nsecs = 0.1
        self.duration_secs = 0.1

        self.angle_theta0_list = [0 for i in range(20)]
        self.angle_theta1_list = [0 for i in range(20)]

        self.speed=[0,0]


        while not rospy.is_shutdown():
            
                #self.force_output = self.palm_force_calculator.output([1.33, 2.11, 1.5, 1.5], [-0.1, -0.1], [3.5, -0.8])
            self.force_output = self.palm_force_calculator.output([self.theta0, self.theta1, 1.5, 1.5],[self.speed[0], self.speed[1]],[3.8, -1.22])

            self.force_output_data = self.force_output
            
            #self.force_output_data = [self.speed[0], self.speed[1]]
            self.palm_controller_data_send.data = self.force_output_data

            palm_force_controller.publish(self.palm_controller_data_send)        


            #palm_controller_data_send.data = [1,2]

            #for i in range (0,2):
            

            '''
            self.output_file = file("data.txt", 'a+')
            self.inp = self.data_array
            self.out = self.force_output
            
            for i in range(0, len(self.inp)):
                self.output_file.write(str(self.inp[i]))
                self.output_file.write(" ")

            self.output_file.write("result:")
            for j in range(0, len(self.out)):
                self.output_file.write(str(self.out[j]))
                self.output_file.write("  ")

            self.output_file.write(" \n")
            self.output_file.close
            '''
           
            rate.sleep()
           

            #self.duration_nsecs = self.time_end.nsecs - self.time_begin.nsecs
            #self.duration_secs = self.time_end.secs - self.time_begin.secs            



        rospy.spin()



    def palm_displacement_velocity_call_back(self,msg):

        #joint_value_from_sensor = Float64MultiArray()
        self.data_received.position = msg.position
    
        for i in range(0, len(self.data_received.position)):
            self.data_array[i] = self.data_received.position[i]

        self.theta0 = self.data_array[0] * 0.01745 + 3.1415
        self.theta1 = -self.data_array[1] * 0.01745

        self.old_value_theta0 = self.sum_list_value(self.angle_theta0_list)
        self.old_value_theta1 = self.sum_list_value(self.angle_theta1_list)

        del self.angle_theta0_list[0]
        del self.angle_theta1_list[0]

        self.angle_theta0_list.append(self.theta0)
        self.angle_theta1_list.append(self.theta1)

        self.new_value_theta0 = self.sum_list_value(self.angle_theta0_list)
        self.new_value_theta1 = self.sum_list_value(self.angle_theta1_list)

        self.delta_theta0 = self.new_value_theta0 - self.old_value_theta0
        self.delta_theta1 = self.new_value_theta1 - self.old_value_theta1


        self.speed[0] = self.delta_theta0 / 0.003
        self.speed[1] = self.delta_theta1 / 0.003





    def sum_list_value(self, list_value):
        sum_value = 0
        for i in range(len(list_value)):
            sum_value = sum_value + list_value[i]
        return sum_value / 20








   
if __name__ == '__main__':
    try:
        palm_stability_calculator()
    except rospy.ROSInterruptException: pass
