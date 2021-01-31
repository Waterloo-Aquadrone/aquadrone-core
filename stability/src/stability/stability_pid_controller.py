import rospy
from simple_pid import PID
import numpy as np

from geometry_msgs.msg import Pose, Wrench
from aquadrone_msgs.msg import StabilityTarget, SubState

from aquadrone_math_utils.angle_math import normalize_angle
from aquadrone_math_utils.orientation_math import quaternion_to_euler
from scipy.spatial.transform import Rotation


class StabilityPIDController:
    @staticmethod
    def normalize_angular_error(angle):
        angle = normalize_angle(angle)
        return angle if angle < np.pi else angle - 2 * np.pi

    def __init__(self, rate=None):
        if rate is None:
            self.rate = rospy.Rate(10)

        self.pids = []
        for axis in 'xyz':
            Kp = rospy.get_param('/stability/' + axis + '/Kp')
            Ki = rospy.get_param('/stability/' + axis + '/Ki')
            Kd = rospy.get_param('/stability/' + axis + '/Kd')

            pid = PID(Kp, Ki, Kd, sample_time=None)
            pid.output_limits = (-50, 50)
            pid.setpoint = 0  # all target positions initialized to 0
            self.pids.append(pid)

        for angle in ['roll', 'pitch', 'yaw']:
            Kp = rospy.get_param('/stability/' + angle + '/Kp')
            Ki = rospy.get_param('/stability/' + angle + '/Ki')
            Kd = rospy.get_param('/stability/' + angle + '/Kd')

            pid = PID(Kp, Ki, Kd, sample_time=None)
            pid.output_limits = (-50, 50)
            pid.setpoint = 0  # all target angles initialized to 0
            pid.error_map = StabilityPIDController.normalize_angular_error
            self.pids.append(pid)

        self.position = np.array([0, 0, 0])
        self.orientation = np.array([0, 0, 0, 1])

        self.depth_sub = rospy.Subscriber("/stability_target", StabilityTarget, callback=self.goal_cb)
        self.state_sub = rospy.Subscriber('/state_estimation', SubState, self.state_cb)
        self.w_pub = rospy.Publisher('/stability_command', Wrench, queue_size=1)

    def goal_cb(self, msg):
        if msg.pose.position.z < 0:
            print('Warning: z < 0 corresponds to above water!')

        position_target = msg.pose.position
        for pid, command, target in zip(self.pids[:3], msg.commands[:3],
                                        [position_target.x, position_target.y, position_target.z]):
            if command:
                pid.setpoint = target

        orientation_target_quat = msg.pose.orientation
        orientation_target_rpy = quaternion_to_euler(np.array([orientation_target_quat.x, orientation_target_quat.y,
                                                               orientation_target_quat.z, orientation_target_quat.w]))
        for pid, command, target in zip(self.pids[3:], msg.commands[3:], orientation_target_rpy):
            if command:
                pid.setpoint = target

    def state_cb(self, msg):
        self.position = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.orientation = np.array([msg.orientation_quat.x, msg.orientation_quat.y,
                                     msg.orientation_quat.z, msg.orientation_quat.w])

    def run(self):
        control = Wrench()
        while not rospy.is_shutdown():
            orientation_rpy = quaternion_to_euler(self.orientation)
            absolute_force = np.array([pid(pos) for pid, pos in zip(self.pids[:3], self.position)])
            absolute_torque = np.array([pid(orientation) for pid, orientation in zip(self.pids[3:], orientation_rpy)])

            tf = Rotation.from_quat(self.orientation).inv().as_matrix()
            relative_force = np.dot(tf, absolute_force)
            relative_torque = np.dot(tf, absolute_torque)

            for i, var in enumerate('xyz'):
                setattr(control.force, var, relative_force[i])
                setattr(control.torque, var, relative_torque[i])
            self.w_pub.publish(control)

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break
