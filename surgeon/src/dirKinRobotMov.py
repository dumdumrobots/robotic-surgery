#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from markers import *
from robotFunctions import *

q_recieved = [0.0, 0.0, 0.0]

def anglesRecieved(data):
    global q_recieved

    qArray = data.position
    q2 = qArray[2]/2 + qArray[3]/2

    q_recieved = [qArray[0], qArray[1], q2]

#Setup
rospy.init_node("robotMovement")

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['YELLOW'])

# Joint names
jnames = ['Prismatic_1', 'Rotational_1', 'Rotational_2', 'Rotational_3']

rospy.Subscriber("joint_states",JointState,anglesRecieved)


# Desired position
pd = np.array([0, 0, 0])
# Initial Configuration
ang0 = np.array([0.0, 0.0, 0.0])
# Inverse Kinematics
ang = ikine(pd, ang0)

# End effector from the base
T = fkine_sci(ang)

print("\nJoint Configuration: \n" + str(q_recieved) + "\n")
print("Transformation from base to end effector\n ")
print(np.round(T, 3))
print("\n")

bmarker.xyz(T[0:3,3])
# Reached position showed by yellow marker
bmarker_des.xyz(pd)

rate = rospy.Rate(100)

while not rospy.is_shutdown():

	rospy.Subscriber("joint_states",JointState,anglesRecieved)

	T = fkine_sci(q_recieved)

	q = ikine(T[0:3,3], q_recieved)
	Tn = fkine_sci(q)

	bmarker.xyz(T[0:3,3])

	bmarker_des.xyz(Tn[0:3,3])

	bmarker.publish()
	bmarker_des.publish()
	rate.sleep()
