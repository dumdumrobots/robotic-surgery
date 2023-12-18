#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32MultiArray

def update_current_position(msg):
        global prism_joint__current_position
        prism_joint__current_position = msg.data

def main():
    global prism_joint__current_position
    prism_joint__current_position = 0

    prism_joint__goal_position = 0
    prism_joint__goal_velocity = 0

    prism_joint__gp_msg = Float32()
    prism_joint__gv_msg = Float32()

    rospy.init_node("speed_test")

    freq = 200
    rate = rospy.Rate(freq)
    
    prism_joint__gp_pub = rospy.Publisher("prism_joint__goal_position",Float32, queue_size=10)
    prism_joint__gv_pub = rospy.Publisher("prism_joint__goal_velocity",Float32, queue_size=10)

    prism_join__cp_sub = rospy.Subscriber("prism_joint__current_position",Float32, update_current_position)

    #rot_joints__gp_pub = rospy.Publisher("rot_joints__goal_position",Int32MultiArray, queue_size=10)

    while not rospy.is_shutdown():

        prism_joint__error_position = prism_joint__current_position - prism_joint__goal_position
        print("Position Error: {0} \n".format(prism_joint__error_position))

        if(abs(prism_joint__error_position) <= 0.01):
            print("Prismatic Joint\n")
            prism_joint__goal_position = float(input("Goal Position: "))
            prism_joint__goal_velocity = float(input("Goal Velocity: "))

            prism_joint__gp_msg.data = prism_joint__goal_position
            prism_joint__gv_msg.data = prism_joint__goal_velocity

            prism_joint__gp_pub.publish(prism_joint__gp_msg)
            prism_joint__gv_pub.publish(prism_joint__gv_msg)

        rate.sleep()

if __name__ == '__main__':
    main()

    