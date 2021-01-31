import rospy
from simple_pid import PID
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

        self.pids = []
        for angle in ['roll', 'pitch', 'yaw']:
            Kp = rospy.get_param('/stability/' + angle + '/Kp')
            Ki = rospy.get_param('/stability/' + angle + '/Ki')
            Kd = rospy.get_param('/stability/' + angle + '/Kd')

            pid = PID(Kp, Ki, Kd)
            pid.output_limits = (-50, 50)
            pid.setpoint = 0  # all target angles initialized to 0
            pid.error_map = OrientationPIDController.normalize_angular_error
            self.pids.append(pid)

        self.quaternion = np.array([0, 0, 0, 1])
        self.orientation = np.array([0, 0, 0])  # roll, pitch, yaw
        rospy.Subscriber("/orientation_target", Vector3, callback=self.goal_cb)
        rospy.Subscriber('/state_estimation', SubState, callback=self.state_cb)
        self.pub = rospy.Publisher('/stability_command', Wrench, queue_size=1)

    def goal_cb(self, msg):
        for pid, target in zip(self.pids, [msg.x, msg.y, msg.z]):
            pid.setpoint = target

    def state_cb(self, msg):
        self.quaternion = np.array([msg.orientation_quat.x, msg.orientation_quat.y,
                                    msg.orientation_quat.z, msg.orientation_quat.w])
        self.orientation = np.array([msg.orientation_RPY.x, msg.orientation_RPY.y, msg.orientation_RPY.z])

    def run(self):
        control = Wrench()
        while not rospy.is_shutdown():
            absolute_torque = np.array([pid(orientation) for pid, orientation in zip(self.pids, self.orientation)])
            relative_torque = np.dot(Rotation.from_quat(self.quaternion).inv().as_matrix(), absolute_torque)
            for i, var in enumerate(['x', 'y', 'z']):
                setattr(control.torque, var, relative_torque[i])
            self.pub.publish(control)

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break
