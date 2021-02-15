import rospy


def ros_time():
    return rospy.Time.now().to_sec()
