import rospy
from simple_pid import PID
import numpy as np

from geometry_msgs.msg import Vector3, Wrench

from aquadrone_msgs.msg import SubState

from scipy.spatial.transform import Rotation


class MovementPIDController:
    def __init__(self, rate=None):
        if rate is None:
            self.rate = rospy.Rate(10)

        self.pids = []
        for axis in 'xy':
            Kp = rospy.get_param('/stability/' + axis + '/Kp')
            Ki = rospy.get_param('/stability/' + axis + '/Ki')
            Kd = rospy.get_param('/stability/' + axis + '/Kd')

            pid = PID(Kp, Ki, Kd)
            pid.output_limits = (-50, 50)
            pid.setpoint = 0  # all target angles initialized to 0
            self.pids.append(pid)

        self.quaternion = np.array([0, 0, 0, 1])
        self.position = np.array([0, 0, 0])

        rospy.Subscriber("/movement_target", Vector3, callback=self.goal_cb)
        rospy.Subscriber('/state_estimation', SubState, callback=self.state_cb)
        self.pub = rospy.Publisher('/movement_command', Wrench, queue_size=1)

    def goal_cb(self, msg):
        for pid, target in zip(self.pids, [msg.x, msg.y]):
            pid.setpoint = target

    def state_cb(self, msg):
        self.quaternion = np.array([msg.orientation_quat.x, msg.orientation_quat.y,
                                    msg.orientation_quat.z, msg.orientation_quat.w])
        self.position = np.array([msg.position.x, msg.position.y])

    def run(self):
        control = Wrench()
        while not rospy.is_shutdown():
            absolute_force = np.array([self.pids[0](self.position[0]), self.pids[1](self.position[1]), 0])
            relative_force = np.dot(Rotation.from_quat(self.quaternion).inv().as_matrix(), absolute_force)
            for var, value in zip('xyz', relative_force):
                setattr(control.force, var, value)
            self.pub.publish(control)

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break
