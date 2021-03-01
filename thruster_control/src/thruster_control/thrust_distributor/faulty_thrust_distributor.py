import rospy
from opensimplex import OpenSimplex
from scipy import stats

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from aquadrone_msgs.msg import MotorControls
from aquadrone_math_utils.ros_utils import ros_time


class FaultyThrustDistributor:
    """
    Receives commands with a list of thrusts for each thruster, and distributes them in separate topics
    Works for both simulated and real thrusters.
    """
    @staticmethod
    def noise_function(mean=0, std=1, velocity=5, seed=None):
        """
        Creates a function that generates a smoothly varying random variable with the given mean and standard deviation.
        :param mean: The mean of the output variable. This describes the average value of the output.
        :param std: The standard deviation of the output variable.
                    This describes the magnitude of the deviations from the mean.
        :param velocity: The scale of the variations relative to the scale of the input variable.
                         Larger values correspond to more sporadically changing outputs.
                         This can be thought of as the rate at which the output moves in the simplex noise space with
                         respect to the rate at which the input changes.
        :param seed: The random seed to use. Defaults to the current time in microseconds.
        """
        if seed is None:
            """
            get time instance and convert to seconds
            """
            seed = int(ros_time() * 1e6)

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

    def __init__(self, thruster_count, namespace='aquadrone'):
        self.thruster_count = thruster_count
        rospy.Subscriber('motor_command', MotorControls, callback=self.process_thrust_command)

        self.noise_functions = [FaultyThrustDistributor.noise_function(0, 3) for _ in range(thruster_count)]

        self.publishers = []
        for i in range(0, thruster_count):
            pub = rospy.Publisher("/%s/thrusters/%d/input" % (namespace, i), FloatStamped, queue_size=1)
            self.publishers.append(pub)

    def process_thrust_command(self, msg):
        thrusts = msg.motorThrusts
        for i, (thrust, noise_func, publisher) in enumerate(zip(thrusts, self.noise_functions, self.publishers)):
            msg = FloatStamped()
            msg.data = thrust + noise_func(ros_time())
            publisher.publish(msg)

    @staticmethod
    def run():
        rospy.spin()
