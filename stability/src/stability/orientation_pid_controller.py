import rospy
import numpy as np

from geometry_msgs.msg import Vector3, Wrench
from aquadrone_msgs.msg import SubState

from aquadrone_math_utils.angle_math import normalize_angle
from scipy.spatial.transform import Rotation


class OrientationPIDController:
    @staticmethod
    def normalize_angular_error(angle):
        angle = normalize_angle(angle)
        return angle if angle < np.pi else angle - 2 * np.pi

    def __init__(self, rate=None):
        if rate is None:
            self.rate = rospy.Rate(10)

        self.target_rotation = Rotation.identity()
        self.rotation = Rotation.identity()
        self.omega = np.array([0, 0, 0])
        rospy.Subscriber("/orientation_target", Vector3, callback=self.goal_cb)
        rospy.Subscriber('/state_estimation', SubState, callback=self.state_cb)
        self.pub = rospy.Publisher('/stability_command', Wrench, queue_size=1)

    def goal_cb(self, msg):
        self.target_rotation = Rotation.from_euler('ZYX', [msg.z, -msg.y, msg.x])

    def state_cb(self, msg):
        self.rotation = Rotation.from_quat([msg.orientation_quat.w, msg.orientation_quat.x,
                                            msg.orientation_quat.y, msg.orientation_quat.z])
        self.omega = np.array([msg.ang_vel.x, msg.ang_vel.y, msg.ang_vel.z])

    def run(self):
        control = Wrench()
        while not rospy.is_shutdown():
            torque = self.calculate_torque()
            for i, var in enumerate(['x', 'y', 'z']):
                setattr(control.torque, var, torque[i])
            self.pub.publish(control)

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

    def calculate_torque(self):
        quat_error = (self.target_rotation * self.rotation.inv()).as_quat()
        axis_error = quat_error[1:] * (1 if quat_error[0] > 0 else -1)
        # Assumes omega is in absolute frame but should be in relative frame
        relative_torque = -10 * axis_error - 5 * self.rotation.as_matrix() * self.omega
        return relative_torque
