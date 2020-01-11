import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Wrench

from data_structures import * as DS

from aquadrone_msgs.msg import SubState


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


class ROSStateEstimationModule:
    def __init__(self):
        self.sub_state_sub = rospy.Subscriber("/state_estimation", SubState, self.sub_state_callback)
        self.sub_state = SubState()

    def sub_state_callback(self, msg):
        self.sub_state = msg

    def get_submarinne_state(self):

        def make_data_type(msg, type=DS.Vector):
            out = type()
            out.from_msg(msg)

        position = make_data_type(self.sub_state.position)
        velocity = make_data_type(self.sub_state.velocity)
        orientation_quat = make_data_type(self.sub_state.orientation_quat, DS.Quaternion)
        orientation_rpy = make_data_type(self.sub_state.orientation_rpy)
        ang_vel = make_data_type(self.sub_state.ang_vel)

        position_var = make_data_type(self.sub_state.position_variance)
        velocity_var = make_data_type(self.sub_state.velocity_variance)
        orientation_quat_var = make_data_type(self.sub_state.orientation_quat_var, DS.Quaternion)
        orientation_rpy_var = make_data_type(self.sub_state.orientation_RPY_var)
        ang_vel_var = make_data_type(self.sub_state.ang_vel_var)

        out = Submarine(position, velocity, orientation_quat, orientation_rpy, ang_vel)
        out.set_uncertainties(position_var, velocity_var, orientation_quat_var, orientation_rpy_var, ang_vel_var)

        return out