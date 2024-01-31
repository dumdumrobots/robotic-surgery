#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Stepper(object):
    
    def __init__(self):

        self.current_position = 0
        self.goal_position = 0
        self.goal_velocity = 0

        self.error_position = 0
        self.movement_direction = 0

        self.linear_acceleration = 30 #mm/s^2
        self.max_velocity = 15 #mm/s

        self.acceleration_time = self.max_velocity / self.linear_acceleration

        self.continuous_time = 0

        self.dt = 0.01
        self.elapsed_time = 0

        self.gp_pub = rospy.Publisher("prism_joint__goal_position",Float32, queue_size=10)
        self.gv_pub = rospy.Publisher("prism_joint__goal_velocity",Float32, queue_size=10)

        self.cp_sub = rospy.Subscriber("prism_joint__current_position",Float32, self.update_current_position)
        self.gp_sub = rospy.Subscriber("prism_joint__goal_position",Float32, self.update_goal_position)


    def accelerate_stepper(self):
        self.goal_velocity += self.movement_direction * self.linear_acceleration * self.dt
        self.publish_goal_velocity()
        self.update_elapsed_time()

    def continuous_stepper(self):
        self.goal_velocity = self.movement_direction * self.max_velocity
        self.publish_goal_velocity()
        self.update_elapsed_time()

    def decelerate_stepper(self):
        self.goal_velocity += self.movement_direction * -self.linear_acceleration * self.dt
        self.publish_goal_velocity()
        self.update_elapsed_time()

    def update_movement_direction(self):
        if (self.error_position == 0):
            self.movement_direction = 0
        else:
            self.movement_direction = -(self.error_position / abs (self.error_position))

    def update_continuous_time(self):
        self.continuous_time = (abs(self.error_position) / self.max_velocity) - self.acceleration_time

    def publish_goal_velocity(self):
        gv_msg = Float32()
        gv_msg.data = self.goal_velocity
        self.gv_pub.publish(gv_msg)

    def publish_goal_position(self):
        gp_msg = Float32()
        gp_msg.data = self.goal_position
        self.gp_pub.publish(gp_msg)

    def update_current_position(self,msg):
        self.current_position = msg.data

    def update_goal_position(self,msg):
        self.goal_position = msg.data

    def update_error_position(self):
        self.error_position = self.current_position - self.goal_position

    def update_elapsed_time(self):
        self.elapsed_time += self.dt

def main():
    rospy.init_node("stepper_node")
    
    freq = 200
    rate = rospy.Rate(freq)
    dt = 1/freq

    stepper = Stepper()
    stepper.dt = dt

    rospy.loginfo("Stepper enabled and ready to use.")

    while not rospy.is_shutdown():

        if(abs(stepper.error_position) <= 0.01):
            
            stepper.elapsed_time = 0
            stepper.goal_velocity = 0

            stepper.publish_goal_velocity()
            stepper.update_error_position()
            stepper.update_continuous_time()
            stepper.update_movement_direction()
            rate.sleep()
            continue

        if(stepper.elapsed_time <= stepper.acceleration_time):
            stepper.accelerate_stepper()
            stepper.update_error_position()
            stepper.update_movement_direction()
            rate.sleep()
            continue
            
        if(stepper.elapsed_time > stepper.acceleration_time and stepper.elapsed_time < (stepper.acceleration_time + stepper.continuous_time)):
            stepper.continuous_stepper()
            stepper.update_error_position()
            stepper.update_movement_direction()
            rate.sleep()
            continue

        if(stepper.elapsed_time >= (stepper.acceleration_time + stepper.continuous_time)):
            stepper.decelerate_stepper()
            stepper.update_error_position()
            stepper.update_movement_direction()
            rate.sleep()
            continue

    rospy.on_shutdown(quit())

if __name__ == '__main__':
    main()

        


