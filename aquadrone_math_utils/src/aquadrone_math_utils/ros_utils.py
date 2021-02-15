import rospy
from geometry_msgs.msg import Vector3, Quaternion


def ros_time():
    return rospy.Time.now().to_sec()


def make_vector(arr):
    msg = Vector3()
    for var, value in zip('xyz', arr):
        setattr(msg, var, value)
    return msg


def make_quaternion(arr):
    msg = Quaternion()
    for var, value in zip('wxyz', arr):
        setattr(msg, var, value)
    return msg
