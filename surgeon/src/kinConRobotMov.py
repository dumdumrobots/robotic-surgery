#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from markers import *
from robotFunctions import *


def correctAngles(q):
	global real_q
	global isOpen

	if isOpen:
		qt = q[2] + 1.0/12.0*np.pi
		qb = q[2] - 1.0/12.0*np.pi
	else:
		qt = q[2]
		qb = q[2]

	real_q = np.array([q[0], q[1], qt, qb])
	return real_q

def correctVel(dq):
	global real_dq
	real_dq = np.array([dq[0], dq[1], dq[2], dq[2]])
	return real_dq


#Setup node
rospy.init_node("robotMovement")

#Setup marker
bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['YELLOW'])

#Setup files
fxcurrent = open("/tmp/xcurrent.txt", "w")           	 
fxdesired = open("/tmp/xdesired.txt", "w")
fq = open("/tmp/q.txt", "w")
fdq = open("/tmp/dq.txt", "w")



# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
pub_vel = rospy.Publisher('prism_joint_vel', Float32, queue_size=10)


# Joint names
jnames = ['Prismatic_1', 'Rotational_1', 'Rotational_2', 'Rotational_3']


# Desired position
xd = np.array([0, 0, 9.253]) #Initial [0, 0, 9.253]
# Initial configuration 
q0 = np.array([0.0, 0.0, 0.0])
real_q = np.array([0.0, 0.0, 0.0, 0.0])

#Geometry
sci_radius = 1.2328 # 12.328cm

# End effector from the base
T0 = fkine_sci(q0)
x0 = T0[0:3,3]

# Initial joint configuration
q = copy(q0)
x = copy(x0)

vel = 0

k = 0.5

# Red marker shows end-effector position
bmarker.xyz(x0)

# Green marker shows desired position
bmarker_des.xyz(xd)

# JointState message
jstate = JointState()
# Message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Initial joint configuration
jstate.position = q

pub.publish(jstate)


#Prismatic message
pstate = Float32()
pstate.data = vel

pub_vel.publish(pstate)


# Frequency (in Hz) and control period
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)

#Scissor initial configuration
isOpen = False
actionOpen = False

while not rospy.is_shutdown():

	pub.publish(jstate)

	xd = np.array([float(input("x: ")),float(input("y: ")),float(input("z: "))])
	actionOpen = bool(input("Open scissors?: "))

	#Setup files
	fxcurrent = open("/tmp/xcurrent.txt", "w")           	 
	fxdesired = open("/tmp/xdesired.txt", "w")
	fq = open("/tmp/q.txt", "w")
	fdq = open("/tmp/dq.txt", "w")

	fqtb = open("/tmp/qtb.txt", "w")
	fdqtb = open("/tmp/dqtb.txt", "w")

	x = fkine_sci(q)[0:3,3]

	e = x - xd

	while np.linalg.norm(e) >= 0.01:

		#print(np.linalg.norm(e))
		#print("\n")

		jstate.header.stamp = rospy.Time.now()

		#Main Articulations
		
		x = fkine_sci(q)[0:3,3]
		e = x - xd

		de = -k*e

		dq = np.linalg.pinv(jacobian_position(q)).dot(de)

		nq = copy(q)

		q = nq + dt*dq

		# Publish
		real_q = correctAngles(q)
		real_dq = correctVel(dq)

		pstate.data = real_dq[0]

		jstate.position = real_q
		jstate.velocity = real_dq

		# Log values
		fxcurrent.write(str(x[0]) +' '+ str(x[1]) +' '+str(x[2]) +'\n')
		fxdesired.write(str(xd[0]) +' '+ str(xd[1])+' '+str(xd[2]) +'\n')

		fq.write(str(q[0]) +" "+ str(q[1]) +" "+ str(q[2]) +"\n")
		fdq.write(str(dq[0]) +" "+ str(dq[1]) +" "+ str(dq[2]) +"\n")

		#Publish

		pub.publish(jstate)
		pub_vel.publish(pstate)


		bmarker_des.xyz(xd)
		bmarker.xyz(x)
		rate.sleep()

	qt = copy(real_q[2])
	qb = copy(real_q[3])

	if actionOpen and not(isOpen):
		qtd = qt + 1.0/12.0*np.pi
		qbd = qb - 1.0/12.0*np.pi

		isOpen = True
		print("Scissors will OPEN soon...")

	elif not(actionOpen) and isOpen:
		qtd = qt - 1.0/12.0*np.pi
		qbd = qb + 1.0/12.0*np.pi

		isOpen = False
		print("Scissors will CLOSE soon...")

	else:
		qtd = qt
		qbd = qb

	print("State: ", isOpen)
	print("Action: ", actionOpen)

	xqt = sci_kin(qt)
	xqtd = sci_kin(qtd)

	xqb = sci_kin(qb)
	xqbd = sci_kin(qbd)

	x_qt_e = xqt - xqtd
	x_qb_e = xqb - xqbd

	while np.linalg.norm(x_qb_e) >= 0.01 or np.linalg.norm(x_qt_e) >= 0.01:

		#print(np.linalg.norm(x_qt_e))
		#print(np.linalg.norm(x_qb_e))
		#print("\n")

		qt = copy(real_q[2])
		qb = copy(real_q[3])

		jstate.header.stamp = rospy.Time.now()

		#Top scissor movement

		xqt = sci_kin(qt)

		x_qt_e = xqt - xqtd

		dx_qt_e = -k*x_qt_e

		dqt = np.linalg.pinv(jacobian_sci_pos(qt)).dot(dx_qt_e)

		qt_past = copy(qt)

		qt = qt_past + dt*dqt[0,0]

		#Bottom scissor movement

		xqb = sci_kin(qb)

		x_qb_e = xqb - xqbd

		dx_qb_e = -k*x_qb_e

		dqb = np.linalg.pinv(jacobian_sci_pos(qb)).dot(dx_qb_e)

		qb_past = copy(qb)

		qb = qb_past + dt*dqb[0,0]

		#Log Values

		fqtb.write(str(qt) +" "+ str(qb) +"\n")
		fdqtb.write(str(dqt[0,0]) +" "+ str(dqb[0,0]) +"\n")

		#Publish
		real_q[2] = copy(qt)
		real_q[3] = copy(qb)

		jstate.position = real_q

		pub.publish(jstate)
		bmarker_des.xyz(xd)
		bmarker.xyz(x)
		rate.sleep()

	fxcurrent.close()
	fxdesired.close()

	fq.close()
	fdq.close()

	fqtb.close()
	fdqtb.close()
	

	rate.sleep()
