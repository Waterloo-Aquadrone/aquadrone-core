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

        self.k_ps = np.array([rospy.get_param('/stability/' + angle + '/Kp') for angle in ['roll', 'pitch', 'yaw']])
        self.k_ds = np.array([rospy.get_param('/stability/' + angle + '/Kd') for angle in ['roll', 'pitch', 'yaw']])

        self.target_rotation = Rotation.identity()
        self.rotation = Rotation.identity()
        self.omega = np.array([0, 0, 0])
        rospy.Subscriber("/orientation_target", Vector3, callback=self.goal_cb)
        rospy.Subscriber('/state_estimation', SubState, callback=self.state_cb)
        self.pub = rospy.Publisher('/stability_command', Wrench, queue_size=1)

    def goal_cb(self, msg):
        self.target_rotation = Rotation.from_euler('ZYX', [msg.z, -msg.y, msg.x])

    def state_cb(self, msg):
        self.rotation = Rotation.from_quat([msg.orientation_quat.x, msg.orientation_quat.y,
                                            msg.orientation_quat.z, msg.orientation_quat.w])
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
        x, y, z, w = quat_error
        axis_error = np.array([x, y, z]) * (1 if w > 0 else -1)
        # Assumes omega is in absolute frame but should be in relative frame
        relative_torque = self.k_ps * axis_error - self.k_ds * self.rotation.apply(self.omega)
        absolute_torque = self.rotation.inv().apply(relative_torque)
        print(f'Target: {self.target_rotation.as_quat()}, current: {self.rotation.as_quat()}, torque: {absolute_torque}')
        return absolute_torque
