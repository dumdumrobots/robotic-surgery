#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32MultiArray

def direction_from_error(x):
   if (x == 0): 
       return 0
   else: 
       return -(x / abs(x))

def update_current_position(msg):
        global prism_joint__current_position
        prism_joint__current_position = msg.data

def main():
    global prism_joint__current_position
    prism_joint__current_position = 0

    prism_joint__goal_position = 0
    prism_joint__goal_velocity = 0

    pj__acceleration = 30 # mm/s^2
    pj__max_velocity = 15 # mm/s

    pj__acceleration_time = pj__max_velocity / pj__acceleration #s

    pj_continuous_time = 0

    prism_joint__gp_msg = Float32()
    prism_joint__gv_msg = Float32()

    rospy.init_node("speed_test")

    freq = 100
    rate = rospy.Rate(freq)

    dt = 1/freq
    start_time = 0
    diff_time = 0 
    elapsed_time = 0
    
    prism_joint__gp_pub = rospy.Publisher("prism_joint__goal_position",Float32, queue_size=10)
    prism_joint__gv_pub = rospy.Publisher("prism_joint__goal_velocity",Float32, queue_size=10)

    prism_join__cp_sub = rospy.Subscriber("prism_joint__current_position",Float32, update_current_position)

    while not rospy.is_shutdown():

        prism_joint__error_position = prism_joint__current_position - prism_joint__goal_position
        prism_joint__direction = direction_from_error(prism_joint__error_position)

        print("Current Position: {0} \n".format(prism_joint__current_position))
        print("Goal Velocity: {0} \n".format(prism_joint__goal_velocity))
        print("Position Error: {0} \n".format(prism_joint__error_position))
        print("Elapsed Time: {0} \n".format(elapsed_time))
        print("Continuous Time: {0} \n".format(pj_continuous_time))


        if(abs(prism_joint__error_position) <= 0.01):

            elapsed_time = 0

            prism_joint__goal_position = float(input("Goal Position: "))
            prism_joint__goal_velocity = 0;

            prism_joint__error_position = prism_joint__current_position - prism_joint__goal_position
            pj_continuous_time = (abs(prism_joint__error_position) / pj__max_velocity) - pj__acceleration_time


            prism_joint__gp_msg.data = prism_joint__goal_position
            prism_joint__gv_msg.data = prism_joint__goal_velocity

            prism_joint__gp_pub.publish(prism_joint__gp_msg)
            prism_joint__gv_pub.publish(prism_joint__gv_msg)

            rate.sleep()
            continue

        if(elapsed_time <= pj__acceleration_time):
             
            prism_joint__goal_velocity += prism_joint__direction * pj__acceleration * dt
            prism_joint__gv_msg.data = prism_joint__goal_velocity
            prism_joint__gv_pub.publish(prism_joint__gv_msg)

            elapsed_time += dt
            rate.sleep()
            continue

        if(elapsed_time > pj__acceleration_time and elapsed_time < (pj__acceleration_time + pj_continuous_time)):
            
            prism_joint__goal_velocity = prism_joint__direction * pj__max_velocity
            prism_joint__gv_msg.data = prism_joint__goal_velocity
            prism_joint__gv_pub.publish(prism_joint__gv_msg)

            elapsed_time += dt
            rate.sleep()
            continue

        if(elapsed_time >= (pj__acceleration_time + pj_continuous_time)):
             
            prism_joint__goal_velocity += prism_joint__direction * -pj__acceleration * dt
            prism_joint__gv_msg.data = prism_joint__goal_velocity
            prism_joint__gv_pub.publish(prism_joint__gv_msg)

            elapsed_time += dt
            rate.sleep()
            continue

if __name__ == '__main__':
    main()

    