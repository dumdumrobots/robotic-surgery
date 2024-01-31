#!/usr/bin/env python3
import rospy
import numpy as np
from instrument_class import Instrument


def main():

    rospy.init_node("instrument_node")
    freq = 200
    rate = rospy.Rate(freq)

    instrument = Instrument()

    rospy.loginfo("Kinematic Test for Instrument")

    while not rospy.is_shutdown():

        instrument.desired_end_position = np.array([float(input("x: ")), 
                                                    float(input("y: ")),
                                                    float(input("z: "))])

        instrument.update_forward_kinematics()
        instrument.update_inverse_kinematics()
        instrument.update_error()

        while np.linalg.norm(instrument.error_position) >= 0.0001:

            instrument.update_forward_kinematics()
            instrument.update_shown_marker()
            instrument.update_joint_states()
            
            instrument.publish_prism_joint_goal_position()

            instrument.update_error()

    rospy.on_shutdown()

if __name__ == '__main__':
    main()