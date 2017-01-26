import numpy as np
import math as m


class Rotation:
    # Rotation about  axis
    def rotation_y(self, y_Angle):
        #y_Angle = m.radians(y_Angle)
        return np.array([[m.cos(y_Angle), 0.0, m.sin(y_Angle)],
                         [0.0, 1.0, 0.0],
                         [-m.sin(y_Angle), 0.0, m.cos(y_Angle)]], dtype = float)

    def rotation_x(self, x_Angle):
        #x_Angle = m.radians(x_Angle)
        return np.array([[1.0, 0.0, 0.0],
                         [0.0, m.cos(x_Angle), -m.sin(x_Angle)],
                         [0.0, m.sin(x_Angle), m.cos(x_Angle)]], dtype = float)

    def rotation_z(self, z_Angle):
        #z_Angle = m.radians(z_Angle)
        return np.array([[m.cos(z_Angle), -m.sin(z_Angle), 0.0],
                         [m.sin(z_Angle), m.cos(z_Angle), 0.0],
                         [0.0, 0.0, 1.0]], dtype = float)

    def skew(self, vector):
        return np.array([[0.0, -vector[2], vector[1]],
                         [vector[2], 0.0, -vector[0]],
                         [-vector[1], vector[0], 0.0]], dtype = float)

    
class Palm_solver: # function [varphi1, varphi3] = palm_mapping(theta1, theta2)

    def __init__(self):

        # Arc angle of links 1, 2, 3, 4, 5
        self.l1 = m.radians(120)
        self.l2 = m.radians(42)
        self.l3 = m.radians(80)
        self.l4 = m.radians(80)
        self.l5 = m.radians(42)
        
        self.control = -1.0
        
    def passive_joints(self, theta1, theta2):
        # Import parameters
        self.theta1 = theta1
        self.theta2 = theta2
        self.A51 = m.pi - self.theta2
        self.A12 = self.theta1
        
        # angles related with l5 and l1
        self.l51 = m.acos(m.cos(self.l5) * m.cos(self.l1) + m.sin(self.l5) * m.sin(self.l1) * m.cos(self.A51))
        self.A1_51 = np.sign(m.sin(self.A51)) * m.acos((m.cos(self.l5) - m.cos(self.l1) * m.cos(self.l51)) /
                                                       (m.sin(self.l1) * m.sin(self.l51)))
        self.A51_5 = np.sign(m.sin(self.A51)) * m.acos((m.cos(self.l1) - m.cos(self.l51) * m.cos(self.l5)) /
                                                       (m.sin(self.l51) * m.sin(self.l5)))
        self.A51_2 = self.A12 - self.A1_51

        # angles related with l1 and l2
        self.l12 = m.acos(m.cos(self.l1) * m.cos(self.l2) + m.sin(self.l1) * m.sin(self.l2) * m.cos(self.A12))
        self.A12_1 = np.sign(m.sin(self.A12)) * m.acos((m.cos(self.l2) - m.cos(self.l12) * m.cos(self.l1)) /
                                                       (m.sin(self.l12) * m.sin(self.l1)))
        self.A2_12 = np.sign(m.sin(self.A12)) * m.acos((m.cos(self.l1) - m.cos(self.l2) * m.cos(self.l12)) /
                                                       (m.sin(self.l2) * m.sin(self.l12)))
        self.A5_12 = self.A51 - self.A12_1

        # angles related with l3 and l4
        self.l34 = m.acos(m.cos(self.l51) * m.cos(self.l2) + m.sin(self.l51) * m.sin(self.l2) * m.cos(self.A51_2))
        self.A34 = self.control * m.acos((m.cos(self.l34) - m.cos(self.l3) * m.cos(self.l4)) /
                                         (m.sin(self.l3) * m.sin(self.l4)))
                                         # here to choose the upper or lower sphere by "control"
        self.A34_3 = m.asin((m.sin(self.A34) * m.sin(self.l4)) / m.sin(self.l34))
        self.A4_34 = m.asin((m.sin(self.A34) * m.sin(self.l3)) / m.sin(self.l34))

        # angles related with l12, l34 and l51
        self.A12_34 = np.sign(m.sin(self.A5_12)) * m.acos((m.cos(self.l5) - m.cos(self.l12) * m.cos(self.l34)) /
                                                          (m.sin(self.l12) * m.sin(self.l34)))
        self.A34_51 = np.sign(m.sin(self.A51_2)) * m.acos((m.cos(self.l2) - m.cos(self.l51) * m.cos(self.l34)) /
                                                          (m.sin(self.l51) * m.sin(self.l34)))

        # joint angles varphi1, varphi2 and varphi2
        self.varphi1 = np.real(self.A2_12 + self.A12_34 + self.A34_3)
        self.varphi3 = np.real(self.A4_34 + self.A34_51 + self.A51_5)

        self.varphi1 = np.remainder(self.varphi1, 2.0*m.pi)
        self.varphi3 = np.remainder(self.varphi3, 2.0*m.pi)

        self.passive_joints_array = np.array([self.varphi1, self.varphi3])
        return self.passive_joints_array

                
class Controller(Rotation, Palm_solver):

    def __init__(self):

        # Initiate Palm_solver
        Palm_solver.__init__(self)

        # Arc angle of links. Unit: rad
        self.l1 = m.radians(120)
        self.l2 = m.radians(42)
        self.l3 = m.radians(80)
        self.l4 = m.radians(80)
        self.l5 = m.radians(42)
        self.lb = m.radians(20)
        self.palmRadius = 47.5 * 0.001    # Unit: m
        self.a0 = 2.1 * 0.001
        self.a1 = 5.0 * 0.001

        # Basis vectors
        self.hat_i = np.array([1.0, 0.0, 0.0])
        self.hat_j = np.array([0.0, 1.0, 0.0])
        self.hat_k = np.array([0.0, 0.0, 1.0])

        # Gravity acceleration. Unit: m/s^2
        self.g = np.array([0.0, 0.0, 9.81])

    ### Parameters obtained from SolidWorks
        
        # Mass of each link. Unit: Kg
        self.m2 = 33.85 * 0.001
        self.m3 = 93.59 * 0.001
        self.m4 = 97.58 * 0.001
        self.m5 = 33.85 * 0.001
        self.m31 = 50.5905 * 0.001
        self.m32 = 58.1705 * 0.001
        self.m41 = 50.5905 * 0.001
        self.m42 = 58.1705 * 0.001

        # Center of mass of each link (defined in the local frame in Solidworks). Unit: m
        self.r2_2c = np.array([-14.61, 0.00, 44.31]) * 0.001
        self.r3_3c = np.array([-24.81, 2.40, 37.77]) * 0.001
        self.r4_4c = np.array([-25.65, 2.32, -36.82]) * 0.001
        self.r5_5c = np.array([14.61, 0.00, 44.31]) * 0.001
        self.r31_31c = np.array([24.23, 0.00, -0.37]) * 0.001
        self.r32_32c = np.array([20.98, -0.08, 0.00]) * 0.001
        self.r41_41c = np.array([24.23, 0.00, 0.37]) * 0.001
        self.r42_42c = np.array([20.98, -0.08, 0.00]) * 0.001

        # Inertia tensors of each link (defined in the local frame in Solidworks). Unit: Kg*m^2
        self.I2_2c = np.array([[1072.71, 0.00, 761.23],
                               [0.00, 4170.73, 0.00],
                               [761.23, 0.0, 4168.31]]) * (10 ** (-9))
        self.I3_3c = np.array([[14659.30, 1930.49, 11122.58],
                               [1930.49, 27476.85, 1537.38],
                               [11122.58, 1537.38, 20770.61]]) * (10 ** (-9))
        self.I4_4c = np.array([[16992.01, 2135.61, -13025.86],
                               [2135.61, 31550.75, -1764.70],
                               [-13025.86, -1764.70, 22679.29]]) * (10 ** (-9))
        self.I5_5c = np.array([[1072.71, 0.00, -761.23],
                               [0.00, 4170.73, 0.00],
                               [-761.23, 0.00, 4168.31]]) * (10 ** (-9))
        self.I31_31c = np.array([[2694.67, -0.00, 437.82],
                                 [-0.00, 17725.87, 0.00],
                                 [437.82, 0.00, 16833.20]]) * (10 ** (-9))
        self.I32_32c = np.array([[1534.73, -129.93, 0.00],
                                 [-129.93, 11744.88, -0.01],
                                 [0.00, -0.01, 11654.12]]) * (10 ** (-9))
        self.I41_41c = np.array([[2694.67, 0.00, -437.82],
                                 [0.00, 17725.87, 0.00],
                                 [-437.82, 0.00, 16833.20]]) * (10 ** (-9))
        self.I42_42c = np.array([[1534.73, -129.93, 0.00],
                                 [-129.93, 11744.88, -0.01],
                                 [0.00, -0.01, 11654.12]]) * (10 ** (-9))

    ### Parameters in the standard local frame 
        
        # Rotation from local frame in Solidworks to local frame in draft paper
        self.R2 = self.rotation_z(-m.pi/2.0)
        self.R3 = self.rotation_z(-m.pi/2.0)
        self.R4 = np.dot(self.rotation_x(-m.pi), self.rotation_z(-m.pi/2.0))
        self.R5 = self.rotation_z(-m.pi/2.0)
        self.R31 = np.eye(3)
        self.R32 = np.eye(3)
        self.R41 = np.eye(3)
        self.R42 = np.eye(3)

        # Center of mass of each link (defined in the local frame in draft paper)
        self.r2_2c = np.dot(self.R2, self.r2_2c)
        self.r3_3c = np.dot(self.R3, self.r3_3c)
        self.r4_4c = np.dot(self.R4, self.r4_4c)
        self.r5_5c = np.dot(self.R5, self.r5_5c)
        self.r31_31c = np.dot(self.R31, self.r31_31c)
        self.r32_32c = np.dot(self.R32, self.r32_32c)
        self.r41_41c = np.dot(self.R41, self.r41_41c)
        self.r42_42c = np.dot(self.R42, self.r42_42c)

        # Inertia tensors of each link (defined in the local frame in draft paper)
        self.I2_2c = np.dot(np.dot(self.R2, self.I2_2c), self.R2.T)
        self.I3_3c = np.dot(np.dot(self.R3, self.I3_3c), self.R3.T)
        self.I4_4c = np.dot(np.dot(self.R4, self.I4_4c), self.R4.T)
        self.I5_5c = np.dot(np.dot(self.R5, self.I5_5c), self.R5.T)
        self.I31_31c = np.dot(np.dot(self.R31, self.I31_31c), self.R31.T)
        self.I32_32c = np.dot(np.dot(self.R32, self.I32_32c), self.R32.T)
        self.I41_41c = np.dot(np.dot(self.R41, self.I41_41c), self.R41.T)
        self.I42_42c = np.dot(np.dot(self.R42, self.I42_42c), self.R42.T)

    def structure(self, signals):

        # Joint angles. Unit: rad
        self.theta1 = signals[0]    
        self.theta2 = signals[1]
        self.varphi1 = self.passive_joints(self.theta1, self.theta2)[0]
        self.varphi3 = self.passive_joints(self.theta1, self.theta2)[1]
        self.phi31 = signals[2]
        self.phi41 = signals[3]
        self.phi32 = (3.0/7.0) * self.phi31 + m.pi/4.0
        self.phi42 = (3.0/7.0) * self.phi41 + m.pi/4.0
                        
    ### Coordinates transformation matrices
        
        self.R0_1 = self.rotation_y(-m.pi/2.0)
        self.R1_1f1 = self.rotation_x(-self.l1/2.0)
        self.R0_1f1 = np.dot(self.R0_1, self.R1_1f1)
        self.R1f1_2 = self.rotation_z(self.theta1 - m.pi) # theta1
        self.R0_2 = np.dot(self.R0_1f1, self.R1f1_2)

        self.R2_2f = self.rotation_x(-self.l2)
        self.R0_2f = np.dot(self.R0_2, self.R2_2f)
        self.R2f_3 = self.rotation_z(self.varphi1 - m.pi) # varphi1
        self.R0_3 = np.dot(self.R0_2f, self.R2f_3)

        self.R3_3f1 = np.dot(self.rotation_x(-self.lb), self.rotation_x(-m.pi/2.0)) # From base frame (at B) of link 3 -
        self.R0_3f1 = np.dot(self.R0_3, self.R3_3f1)                      # to follower frame at P2
        self.R3f1_31 = self.rotation_z(self.phi31 - m.pi/2.) # phi31
        self.R0_31 = np.dot(self.R0_3f1, self.R3f1_31)
        
        self.R31_32 = self.rotation_z(self.phi32) # phi32
        self.R0_32 = np.dot(self.R0_31, self.R31_32)

        self.R1_1f2 = self.rotation_x(self.l1/2.0)
        self.R0_1f2  = np.dot(self.R0_1, self.R1_1f2)
        self.R1f2_5 = self.rotation_z(self.theta2) # theta2
        self.R0_5 = np.dot(self.R0_1f2, self.R1f2_5)

        self.R5_5f = self.rotation_x(self.l5)
        self.R0_5f = np.dot(self.R0_5, self.R5_5f)
        self.R5f_4 = self.rotation_z(m.pi - self.varphi3) # varphi3
        self.R0_4 = np.dot(self.R0_5f, self.R5f_4)

        self.R4_4f1 = np.dot(self.rotation_x(self.lb), self.rotation_x(-m.pi/2.0)) # From base frame (at B) of link 4 -
        self.R0_4f1 = np.dot(self.R0_4, self.R4_4f1)                   # to follower frame at P2
        self.R4f1_41 = self.rotation_z(self.phi41 - m.pi/2) # phi41
        self.R0_41 = np.dot(self.R0_4f1, self.R4f1_41)

        self.R41_42 = self.rotation_z(self.phi42) # phi42
        self.R0_42 = np.dot(self.R0_41, self.R41_42)

    ### Parameters in the standard global frame

        # Center of mass of each link (defined in the global frame in draft paper)
        self.r_2c = np.dot(self.R0_2, self.r2_2c)
        self.r_3c = np.dot(self.R0_3, self.r3_3c)
        self.r_5c = np.dot(self.R0_5, self.r5_5c)
        self.r_4c = np.dot(self.R0_4, self.r4_4c)
        self.r3f1_31c = np.dot(self.a0, self.hat_i) - \
                        np.dot(self.palmRadius, self.hat_j) + \
                        np.dot(self.R3f1_31, self.r31_31c)
        self.r_31c = np.dot(self.R0_3f1, self.r3f1_31c)
        self.r31_32c = np.dot(self.a1, self.hat_i) + np.dot(self.R31_32, self.r32_32c)
        self.r3f1_32c = np.dot(self.a0, self.hat_i) - \
                        np.dot(self.palmRadius, self.hat_j) + \
                        np.dot(self.R3f1_31, self.r31_32c)
        self.r_32c = np.dot(self.R0_3f1, self.r3f1_32c)
        self.r4f1_41c = np.dot(self.a0, self.hat_i) - \
                        np.dot(self.palmRadius, self.hat_j) + \
                        np.dot(self.R4f1_41, self.r41_41c)
        self.r_41c = np.dot(self.R0_4f1, self.r4f1_41c)
        self.r41_42c = np.dot(self.a1, self.hat_i) + np.dot(self.R41_42, self.r42_42c)
        self.r4f1_42c = np.dot(self.a0, self.hat_i) - \
                        np.dot(self.palmRadius, self.hat_j) + \
                        np.dot(self.R4f1_41, self.r41_42c)
        self.r_42c = np.dot(self.R0_4f1, self.r4f1_42c)

        # Inertia tensors of each link (defined in the global frame in draft paper)
        self.I_2c = np.dot(np.dot(self.R0_2, self.I2_2c), self.R0_2.T)
        self.I_3c = np.dot(np.dot(self.R0_3, self.I3_3c), self.R0_3.T)
        self.I_5c = np.dot(np.dot(self.R0_5, self.I5_5c), self.R0_5.T)
        self.I_4c = np.dot(np.dot(self.R0_4, self.I4_4c), self.R0_4.T)
        self.I_31c = np.dot(np.dot(self.R0_31, self.I31_31c), self.R0_31.T)
        self.I_32c = np.dot(np.dot(self.R0_32, self.I32_32c), self.R0_32.T)
        self.I_41c = np.dot(np.dot(self.R0_41, self.I41_41c), self.R0_41.T)
        self.I_42c = np.dot(np.dot(self.R0_42, self.I42_42c), self.R0_42.T)

    ### Jacobian matrices
        
        # Linear velocity Jacobian matrices of each link
        self.Jv2_1 = np.dot(self.skew(np.dot(self.R0_1f1, self.hat_k)), self.r_2c)
        self.Jv2 = np.column_stack((self.Jv2_1,
                                    np.zeros((3, 1)),
                                    np.zeros((3, 1)),
                                    np.zeros((3, 1))))
        self.Jv3_1 = np.dot(self.skew(np.dot(self.R0_1f1, self.hat_k)), self.r_3c)
        self.Jv3_3 = np.dot(self.skew(np.dot(self.R0_2f, self.hat_k)), self.r_3c)
        self.Jv3 = np.column_stack((self.Jv3_1,
                                    np.zeros((3, 1)),
                                    self.Jv3_3,
                                    np.zeros((3, 1))))
        self.Jv5_2 = np.dot(self.skew(np.dot(self.R0_1f2, self.hat_k)), self.r_5c)
        self.Jv5 = np.column_stack((np.zeros((3, 1)),
                                    self.Jv5_2,
                                    np.zeros((3, 1)),
                                    np.zeros((3, 1))))
        self.Jv4_2 = np.dot(self.skew(np.dot(self.R0_1f2, self.hat_k)), self.r_4c)
        self.Jv4_4 = np.dot(self.skew(np.dot(self.R0_5f, self.hat_k)), self.r_4c)
        self.Jv4 = np.column_stack((np.zeros((3, 1)),
                                    self.Jv4_2,
                                    np.zeros((3, 1)),
                                    self.Jv4_4))

        self.Jv31_1 = np.dot(self.skew(np.dot(self.R0_1f1, self.hat_k)), self.r_31c)
        self.Jv31_3 = np.dot(self.skew(np.dot(self.R0_3f1, self.hat_k)), np.dot(self.R0_31, self.r31_31c))
        self.Jv31_5 = np.dot(self.skew(np.dot(self.R0_2f, self.hat_k)), self.r_31c)
        self.Jv31 = np.column_stack((self.Jv31_1,
                                     np.zeros((3, 1)),
                                     self.Jv31_5,
                                     np.zeros((3, 1))))
        self.Jv32_1 = np.dot(self.skew(np.dot(self.R0_1f1, self.hat_k)), self.r_32c)
        self.Jv32_3 = np.dot(self.skew(np.dot(self.R0_3f1, self.hat_k)), np.dot(self.R0_31, self.r31_32c)) + \
                      (3.0/7.0) * np.dot(self.skew(np.dot(self.R0_31, self.hat_k)),
                                         np.dot(self.R0_32, self.r32_32c))
        self.Jv32_5 = np.dot(self.skew(np.dot(self.R0_2f, self.hat_k)), self.r_32c)
        self.Jv32 = np.column_stack((self.Jv32_1,
                                     np.zeros((3, 1)),
                                     self.Jv32_5,
                                     np.zeros((3, 1))))
        self.Jv41_2 = np.dot(self.skew(np.dot(self.R0_1f2, self.hat_k)), self.r_41c)
        self.Jv41_4 = np.dot(self.skew(np.dot(self.R0_4f1, self.hat_k)), np.dot(self.R0_41, self.r41_41c))
        self.Jv41_6 = np.dot(self.skew(np.dot(self.R0_5f, self.hat_k)), self.r_41c)
        self.Jv41 = np.column_stack((np.zeros((3, 1)),
                                     self.Jv41_2,
                                     np.zeros((3, 1)),
                                     self.Jv41_6))
        self.Jv42_2 = np.dot(self.skew(np.dot(self.R0_1f2, self.hat_k)), self.r_42c)
        self.Jv42_4 = np.dot(self.skew(np.dot(self.R0_4f1, self.hat_k)), np.dot(self.R0_41, self.r41_42c)) + \
                      (3.0/7.0) * np.dot(self.skew(np.dot(self.R0_41, self.hat_k)),
                                         np.dot(self.R0_42, self.r42_42c))
        self.Jv42_6 = np.dot(self.skew(np.dot(self.R0_5f, self.hat_k)), self.r_42c)
        self.Jv42 = np.column_stack((np.zeros((3, 1)),
                                     self.Jv42_2,
                                     np.zeros((3, 1)),
                                     self.Jv42_6))

        # Angular velocity Jacobian matrices of each link
        self.Jw2 = np.column_stack((np.dot(self.R0_1f1, self.hat_k),
                                    np.zeros((3, 1)),
                                    np.zeros((3, 1)),
                                    np.zeros((3, 1))))
        self.Jw3 = np.column_stack((np.dot(self.R0_1f1, self.hat_k),
                                    np.zeros((3, 1)),
                                    np.dot(self.R0_2f, self.hat_k),
                                    np.zeros((3, 1))))
        self.Jw5 = np.column_stack((np.zeros((3, 1)),
                                    np.dot(self.R0_1f2, self.hat_k),
                                    np.zeros((3, 1)),
                                    np.zeros((3, 1))))
        self.Jw4 = np.column_stack((np.zeros((3, 1)),
                                    np.dot(self.R0_1f2, self.hat_k),
                                    np.zeros((3, 1)),
                                    np.dot(self.R0_5f, self.hat_k)))
        
        self.Jw31 = np.column_stack((np.dot(self.R0_1f1, self.hat_k),
                                     np.zeros((3, 1)),
                                     np.dot(self.R0_2f, self.hat_k),
                                     np.zeros((3, 1))))
        self.Jw32_3 = np.dot(self.R0_3f1, self.hat_k) + (3.0/7.0) * np.dot(self.R0_31, self.hat_k)
        self.Jw32 = np.column_stack((np.dot(self.R0_1f1, self.hat_k),
                                     np.zeros((3, 1)),
                                     np.dot(self.R0_2f, self.hat_k),
                                     np.zeros((3, 1))))
        self.Jw41 = np.column_stack((np.zeros((3, 1)),
                                     np.dot(self.R0_1f2, self.hat_k),
                                     np.zeros((3, 1)),
                                     np.dot(self.R0_5f, self.hat_k)))
        self.Jw42_4 = np.dot(self.R0_4f1, self.hat_k) + (3.0/7.0) * np.dot(self.R0_41, self.hat_k)
        self.Jw42 = np.column_stack((np.zeros((3, 1)),
                                     np.dot(self.R0_1f2, self.hat_k),
                                     np.zeros((3, 1)),
                                     np.dot(self.R0_5f, self.hat_k)))

    ### Inertial matrix D(q) and gravity effect g(q) of the open chain model 

        # inertial matrices related with translational and rotational energy
        self.cal_T = self.m2 * np.dot(self.Jv2.T, self.Jv2) + \
                     self.m3 * np.dot(self.Jv3.T, self.Jv3) + \
                     self.m4 * np.dot(self.Jv4.T, self.Jv4) + \
                     self.m5 * np.dot(self.Jv5.T, self.Jv5) + \
                     self.m31 * np.dot(self.Jv31.T, self.Jv31) + \
                     self.m32 * np.dot(self.Jv32.T, self.Jv32) + \
                     self.m41 * np.dot(self.Jv41.T, self.Jv41) + \
                     self.m42 * np.dot(self.Jv42.T, self.Jv42)
        self.cal_R = np.dot(np.dot(self.Jw2.T, self.I_2c), self.Jw2) + \
                     np.dot(np.dot(self.Jw3.T, self.I_3c), self.Jw3) + \
                     np.dot(np.dot(self.Jw4.T, self.I_4c), self.Jw4) + \
                     np.dot(np.dot(self.Jw5.T, self.I_5c), self.Jw5) + \
                     np.dot(np.dot(self.Jw31.T, self.I_31c), self.Jw31) + \
                     np.dot(np.dot(self.Jw32.T, self.I_32c), self.Jw32) + \
                     np.dot(np.dot(self.Jw41.T, self.I_41c), self.Jw41) + \
                     np.dot(np.dot(self.Jw42.T, self.I_42c), self.Jw42)

        # Parameter D(q) and g(q) of the open chain model
        self.cal_D = self.cal_T + self.cal_R
        self.cal_g = self.m2 * np.dot(self.Jv2.T, self.g) + \
                     self.m3 * np.dot(self.Jv3.T, self.g) + \
                     self.m4 * np.dot(self.Jv4.T, self.g) + \
                     self.m5 * np.dot(self.Jv5.T, self.g) + \
                     self.m31 * np.dot(self.Jv31.T, self.g) + \
                     self.m32 * np.dot(self.Jv32.T, self.g) + \
                     self.m41 * np.dot(self.Jv41.T, self.g) + \
                     self.m42 * np.dot(self.Jv42.T, self.g)

    ### Inertial matrix D(q) and gravity effect g(q) of the closed chain model
        
        # Position of point C calculated from links 2 and 3
        self.R3_3f2 = self.rotation_x(-self.l3)
        self.r_3f2 = np.dot(np.dot(self.R0_3, self.R3_3f2), self.hat_k) * self.palmRadius
        
        # Position of point C calculated from links 5 and 4
        self.R4_4f2 = self.rotation_x(self.l4)
        self.r_4f2 = np.dot(np.dot(self.R0_4, self.R4_4f2), self.hat_k) * self.palmRadius

        self.Jc = np.column_stack((np.dot(self.skew(np.dot(self.R0_1f1, self.hat_k)), self.r_3f2),
                                   -np.dot(self.skew(np.dot(self.R0_1f2, self.hat_k)), self.r_4f2),
                                   np.dot(self.skew(np.dot(self.R0_2f, self.hat_k)), self.r_3f2),
                                   -np.dot(self.skew(np.dot(self.R0_5f, self.hat_k)), self.r_4f2)))

        self.alpha_q = np.dot(np.column_stack((np.zeros((2, 1)), np.eye(2))), self.Jc)
        self.beta_q = np.column_stack((np.eye(2), np.zeros((2, 2))))
        self.gamma_q = np.concatenate((self.alpha_q, self.beta_q))

        # Transformation matrix from open chain parameters to closed chain parameters
        self.rho = np.dot(np.linalg.inv(self.gamma_q), np.concatenate((np.zeros((2, 2)), np.eye(2))))

        # Parameter D(q) and g(q) of the closed chain model
        self.rm_D = np.dot(np.dot(self.rho.T, self.cal_D), self.rho)
        self.rm_g = np.dot(self.rho.T, self.cal_g)

        return self.rm_D, self.rm_g

    def output(self, signals, dot_signals, desired_signals):
        #---------------- inputs ----------------
        # signals: [theta1, theta2, phi31, phi41]
        # dot_signals: [dot_theta1, dot_theta2]
        # desired_signals: [desired_theta1, desired_theta2]

        # Dynamic parameters
        self.signals = np.array(signals[:2]) 
        self.dot_signals = np.array(dot_signals)
        self.desired_signals = np.array(desired_signals)

        # Inherit the parameter from structure       
        self.rm_D = self.structure(signals)[0]
        self.rm_g = self.structure(signals)[1]
        
        # Feedback variables
        self.signal_errors = self.signals - self.desired_signals
        self.xx = np.concatenate((self.dot_signals, self.signal_errors))

        self.settling_time = 0.15

        # Control parameters k1 and k2
        self.k1 = -7.0 / self.settling_time
        self.k2 = -2.0 * (-self.k1/2.0) ** 2.0

        self.K = np.array([[self.k1, 0.0, self.k2, 0.0],
                           [0.0, self.k1, 0.0, self.k2]])

        # Control signal
        self.u = np.dot(np.dot(self.rm_D, self.K), self.xx) + self.rm_g

        return self.u
        

a_controller = Controller()

# print a_controller.structure([1.33, 2.11, 1.5, 1.5])
print a_controller.output([1.33, 2.11, 1.5, 1.5], [-0.1, -0.1], [3.5, -0.8])




