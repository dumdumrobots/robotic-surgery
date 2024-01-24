#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, Float32

from markers import BallMarker, color
from copy import copy

pi = np.pi

# --- Define an robotic instrument instance
class Instrument(object):

    def __init__(self):

        self.present_q = np.array([0.0, 0.0, 0.0]) # mm / 100
        self.present_end_transform = np.zeros((4,4))

        self.present_end_position = self.present_end_transform[0:3,3]
        self.present_jacobian = np.zeros((3,3))

        self.delta = 0.0001

        self.desired_q = np.array([0.0, 0.0, 0.0])
        self.desired_end_position = np.array([0.0, 0.0, 0.0])

        self.error_q = np.array([0.0, 0.0, 0.0])
        self.error_position = np.array([0.0, 0.0, 0.0])

        self.joints_names = ["world__frame",
                             "frame__platform",
                             "scissor_base__scissor_neck",
                             "scissor_neck__scissor__blade_top",
                             "scissor_base__scissor__blade_bot"]
        
        self.end_effector_marker = BallMarker(color['RED'])
        self.desired_effector_marker = BallMarker(color['GREEN'])

        self.joint_states_pub = rospy.Publisher("joint_states",JointState, queue_size=10)

        self.rot_joints__pp_sub = rospy.Subscriber("rot_joints__present_position",Int32MultiArray,self.update_rot_joints_shown_angles)
        self.prism_joint__cp_sub = rospy.Subscriber("prism_joint__current_position",Float32,self.update_prism_joint_shown_angle)

 
    def dh_transformation(self, d, theta, a, alpha):
       sth = np.sin(theta)
       cth = np.cos(theta)
       sa  = np.sin(alpha)
       ca  = np.cos(alpha)

       T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                     [sth,  ca*cth, -sa*cth, a*sth],
                     [0.0,      sa,      ca,     d],
                     [0.0,     0.0,     0.0,   1.0]])
       return T   

    def position_jacobian(self):

        for i in range(3):

            delta_present_q = copy(self.present_q)

            delta_present_q[i] += self.delta

            delta_T = self.forward_kinematics(delta_present_q)
            delt_X = delta_T[0:3,3]

            self.present_jacobian[0,i] = (delt_X[0] -  self.present_end_position[0])/self.delta
            self.present_jacobian[1,i] = (delt_X[1] -  self.present_end_position[1])/self.delta
            self.present_jacobian[2,i] = (delt_X[2] -  self.present_end_position[2])/self.delta

    def inverse_kinematics(self):
        epsilon=0.001
        max_iter=1000

        q = copy(self.present_q)
        self.position_jacobian()

        for i in range(max_iter):
            T = self.forward_kinematics(q)
            e = self.desired_end_position - T[0:3,3]
            q = q + np.dot(np.linalg.pinv(self.present_jacobian), e)

            if(np.linalg.norm(e) < epsilon):
                break

        self.desired_q = q

    def forward_kinematics(self,q):
        d     = np.array([ q[0] + 0.22378,           0,     0])
        th    = np.array([              0, q[1] + pi/2,  q[2]])
        a     = np.array([              0,      0.0878, 0.135])
        alpha = np.array([           pi/2,        pi/2,     0])
        
        T1 = self.dh_transformation(d[0], th[0], a[0], alpha[0])
        T2 = self.dh_transformation(d[1], th[1], a[1], alpha[1])
        T3 = self.dh_transformation(d[2], th[2], a[2], alpha[2])

        return T1.dot(T2).dot(T3)

    def update_forward_kinematics(self):
        self.present_end_transform = self.forward_kinematics(self.present_q)
        self.present_end_position = self.present_end_transform[0:3,3]

    def update_inverse_kinematics(self):
        self.inverse_kinematics()
        self.desired_end_position = self.forward_kinematics(self.desired_q)[0:3,3]

    def update_error(self):
        self.error_q = self.desired_q - self.present_q
        self.error_position = self.desired_end_position - self.present_end_position 

    def update_rot_joints_shown_angles(self,msg):
        pass

    def update_prism_joint_shown_angle(self,msg):
        self.present_q[0] = msg.data / 100


    def update_shown_marker(self):
        self.end_effector_marker.xyz(self.present_end_position)
        self.desired_effector_marker.xyz(self.desired_end_position)
        self.end_effector_marker.publish()
        self.desired_effector_marker.publish()
        
    def update_joint_states(self):
        
        jstate = JointState()
        jstate.name = self.joints_names
        jstate.header.stamp = rospy.Time.now()

        jstate.position = np.hstack(([0],self.present_q, self.present_q[2]))

        self.joint_states_pub.publish(jstate)






