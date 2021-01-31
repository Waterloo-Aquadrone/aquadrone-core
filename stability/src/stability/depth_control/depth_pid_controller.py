import rospy
from simple_pid import PID
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench

from aquadrone_msgs.msg import SubState

from scipy.spatial.transform import Rotation
import aquadrone_math_utils.orientation_math as OH


class DepthPIDController:
    def __init__(self, Kp=1, Kd=1, Ki=0):
        self.depth = 0

        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        self.loop_rate = 10.0
        self.rate = rospy.Rate(self.loop_rate)
        self.pid = PID(self.Kp, self.Ki, self.Kd)
        self.pid.output_limits = (-50, 50)
        self.pid.sample_time = 1.0 / self.loop_rate 

        # In future, take in state estimation, and not sensor reading
        self.state_sub = rospy.Subscriber('state_estimation', SubState, self.state_cb)

        self.pressure_offset = 100.0
        self.g = 9.8  # m/s^2

        self.w_pub = rospy.Publisher('/depth_command', Wrench, queue_size=1)
        self.depth_sub = rospy.Subscriber("/depth_control/goal_depth", Float64, callback=self.goal_cb)

        # Increasing depth (positive) will be negatively increasing position.z
        self.depth_goal = 3
        self.pid.setpoint = self.depth_goal

        self.quaternion = Rotation.from_matrix(np.identity(3)).as_quat()

    def goal_cb(self, msg):
        if msg.data < 0:
            print('Warning: depth < 0 corresponds to above water!')
        self.depth_goal = msg.data
        self.pid.setpoint = self.depth_goal

    def state_cb(self, msg):
        self.depth = -msg.position.z

        self.quaternion = np.array([msg.orientation_quat.x, msg.orientation_quat.y,
                                    msg.orientation_quat.z, msg.orientation_quat.w])

    def pressure_to_m(self, p):
        return (p - self.pressure_offset) / self.g

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

    def control_loop(self):
        u = -self.pid(self.depth)
        self.publish_wrench(u)
        # print("Goal/Depth = %f/%f" % (self.depth_goal, self.depth))

    def publish_wrench(self, u):
        vec = np.dot(Rotation.from_quat(self.quaternion).as_matrix(), np.array([0, 0, u]))
        # print(vec)

        msg = Wrench()
        msg.force.x = vec[0]
        msg.force.y = vec[1]
        msg.force.z = vec[2]
        self.w_pub.publish(msg)
