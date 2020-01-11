import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Wrench


class ROSControlsModule:
    def __init__(self):
        self.depth_pub = rospy.Publisher("/depth_control/goal_depth", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("orientation_target", Vector3, queue_size=1)
        self.planar_move_pub = rospy.Publisher("/movement_command", Wrench, queue_size=1)


    def set_depth_goal(self, d):
        self.depth_pub.publish(d)

    def set_yaw_goal(self, y):
        target = Vector3()
        target.x = 0
        target.y = 0
        target.z = y
        self.yaw_pub.publish(target)

    def planar_move_command(Fx=0, Fy=0, Tz=0):
        w = Wrench()
        w.force.x = Fx
        w.force.y = Fy
        w.force.z = 0
        w.torque.x = 0
        w.torque.y = 0
        w.torque.z = Tz
        self.planar_move_pub.publish(w)
