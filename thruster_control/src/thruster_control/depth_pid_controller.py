import rospy
from simple_pid import PID

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from sensor_msgs.msg import FluidPressure

from aquadrone_msgs.msg import SubState

class DepthPIDController:

    def __init__(self):
        rospy.init_node('depth_control')
        self.depth = 0

        self.Kp = 5
        self.Kd = 10
        self.Ki = 0.1

        self.loop_rate = 10.0
        self.rate = rospy.Rate(self.loop_rate)
        self.pid = PID(self.Kp, self.Ki, self.Kd)
        self.pid.output_limits = (-50, 50)
        self.pid.sample_time = 1.0 / self.loop_rate 

        # In future, take in state estimation, and not sensor reading
        self.state_sub = rospy.Subscriber('state_estimation', SubState, self.state_cb)

        self.pressure_offset = 100.0
        self.g = 9.8

        self.w_pub = rospy.Publisher('/depth_command', Wrench, queue_size=3)
        self.depth_sub = rospy.Subscriber("/depth_control/goal_depth", Float64, callback=self.goal_cb)

        # Increasing depth (positive) will be negatively increasing position.z
        self.depth_goal=3
        self.pid.setpoint = self.depth_goal

    def goal_cb(self, msg):
        self.depth_goal = msg.data
        self.pid.setpoint = self.depth_goal


    def state_cb(self, msg):
        self.depth = -msg.position.z
        

    def pressure_to_m(self, p):
        return (p - self.pressure_offset) / self.g

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    def control_loop(self):
        u = -self.pid(self.depth)
        self.publish_wrench(u)
        # print("Goal/Depth = %f/%f" % (self.depth_goal, self.depth))


    def publish_wrench(self, u):
        msg = Wrench()
        msg.force.z = u
        self.w_pub.publish(msg)