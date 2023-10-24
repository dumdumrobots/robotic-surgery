import rospy

def main():

    rospy.init_node("node")
    freq = 0.5
    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        rospy.loginfo("INFO")
        rospy.logwarn("WARN")
        rospy.logerr("ERROR")
        rospy.logfatal("FATAL")
        rate.sleep()

if __name__ == '__main__':
    main()