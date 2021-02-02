import rospy
from simple_pid import PID
import numpy as np

from geometry_msgs.msg import Vector3, Wrench

from aquadrone_msgs.msg import SubState

from aquadrone_math_utils.angle_math import normalize_angle
from aquadrone_math_utils.orientation_math import RPY_Matrix


class OrientationPIDController:
    @staticmethod
    def normalize_angular_error(angle):
        angle = normalize_angle(angle)
        return angle if angle < np.pi else angle - 2 * np.pi

    def __init__(self, rate=None):
        if rate is None:
            self.rate = rospy.Rate(10)

        # the pids will be in the relative intrinsic-rotation coordinate frame
        self.pids = []
        for angle in ['roll', 'pitch', 'yaw']:
            Kp = rospy.get_param('/stability/' + angle + '/Kp')
            Ki = rospy.get_param('/stability/' + angle + '/Ki')
            Kd = rospy.get_param('/stability/' + angle + '/Kd')

            pid = PID(Kp, Ki, Kd, sample_time=None)
            pid.output_limits = (-50, 50)
            pid.setpoint = 0  # all target angles initialized to 0
            pid.error_map = OrientationPIDController.normalize_angular_error
            self.pids.append(pid)

        self.rpy = [0, 0, 0]
        rospy.Subscriber("/orientation_target", Vector3, callback=self.goal_cb)
        rospy.Subscriber('/state_estimation', SubState, callback=self.state_cb)
        self.pub = rospy.Publisher('/stability_command', Wrench, queue_size=1)

    def goal_cb(self, msg):
        for pid, target_angle in zip(self.pids, [msg.x, msg.y, msg.z]):
            pid.setpoint = target_angle

    def state_cb(self, msg):
        self.rpy = [msg.orientation_RPY.x, msg.orientation_RPY.y, msg.orientation_RPY.z]

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
        # TODO: use full quaternion based PID, ensure that it works in all cases reliably
        # quat_error = (self.target_rotation * self.rotation.inv()).as_quat()
        # axis_error = quat_error[1:] * (1 if quat_error[0] > 0 else -1)
        # absolute_torque = np.array([pid(orientation) for pid, orientation in zip(self.pids, axis_error)])
        absolute_torque = np.array([pid(angle) for pid, angle in zip(self.pids, self.rpy)])
        relative_torque = np.dot(RPY_Matrix(*self.rpy), absolute_torque)
        return relative_torque
