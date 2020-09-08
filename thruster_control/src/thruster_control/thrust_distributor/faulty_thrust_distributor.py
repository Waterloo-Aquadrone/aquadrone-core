import rospy
import numpy as np
from opensimplex import OpenSimplex
from scipy import stats

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from aquadrone_msgs.msg import MotorControls

def noise_function(mean, std, velocity=1, seed=None):
    """
    Creates a function that generates a smoothly varying random variable with the given mean and standard deviation.
    """
    if seed is None:
        """
        get time instance and convert to seconds
        """
        curr = rospy.Time().now()
        seed = int(curr.to_sec() * 1e6)
        
    noise_generator = OpenSimplex(seed)
    def noise(t):
        """
        Calculates a noise value for the given time.
        """
        noise_value = noise_generator.noise2d(t * velocity, 0)
        cdf_value = (noise_value + 1) / 2
        gaussian_value = stats.norm.ppf(cdf_value, loc=mean, scale=std)
        return gaussian_value
    return noise


class FaultyThrustDistributor:
    """
    Receives commands with a list of thrusts for each thruster, and distributes them in separate topics
    Works for both simulated and real thrusters.
    """
    def __init__(self, thruster_count, namespace='aquadrone'):
        self.thruster_count = thruster_count
        rospy.Subscriber('motor_command', MotorControls, callback=self.process_thrust_command)

        self.noise_functions = [noise_function(0, 3) for _ in range(thruster_count)]

        self.publishers = []
        for i in range(0, thruster_count):
            pub = rospy.Publisher("/%s/thrusters/%d/input" % (namespace, i), FloatStamped, queue_size=1)
            self.publishers.append(pub)


    def process_thrust_command(self, msg):
        thrusts = msg.motorThrusts
        for i, thrust in enumerate(thrusts):
            msg = FloatStamped()
            t = rospy.Time().now()
            msg.data = thrust + self.noise_functions[i](t.to_sec())
            self.publishers[i].publish(msg)

    @staticmethod
    def run():
        rospy.spin()
