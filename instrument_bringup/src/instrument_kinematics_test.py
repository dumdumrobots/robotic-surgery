#!/usr/bin/env python3
import rospy
import numpy as np
from instrument_class import Instrument


def main():

    rospy.init_node("instrument_node")
    freq = 100
    rate = rospy.Rate(freq)

    instrument = Instrument()

    instrument.desired_end_position = np.array([0.0, 0.0, 1.0])
    
    instrument.update_forward_kinematics()
    instrument.update_inverse_kinematics()

    while not rospy.is_shutdown():

        instrument.update_forward_kinematics()
        instrument.update_shown_marker()
        instrument.update_error()
        instrument.update_joint_states()

if __name__ == '__main__':
    main()