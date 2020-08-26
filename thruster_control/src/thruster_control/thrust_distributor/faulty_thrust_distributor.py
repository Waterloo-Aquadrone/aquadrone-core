from opensimplex import OpenSimplex
from scipy import stats

import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from aquadrone_msgs.msg import MotorControls


class FaultyThrustDistributor:
	@staticmethod
	def get_time():
		return rospy.Time.now().to_sec()

	@staticmethod
	def get_noise_model(mean=0, std=1, velocity=5, seed=None):
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
	        seed = int(FaultyThrustDistributor.get_time() * 1e6)
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

    """
    Receives commands with a list of thrusts for each thruster, and distributes them in separate topics
    Works for both simulated and real thrusters.
    """
    def __init__(self, thruster_count, namespace='aquadrone'):
        self.thruster_count = thruster_count
        rospy.Subscriber('motor_command', MotorControls, callback=self.process_thrust_command)

        self.noise_models = [FaultyThrustDistributor.get_noise_model() 
                             for i in range(thruster_count)]

        self.publishers = []
        for i in range(0, thruster_count):
            pub = rospy.Publisher("/%s/thrusters/%d/input" % (namespace, i), FloatStamped, queue_size=1)
            self.publishers.append(pub)

    def process_thrust_command(self, msg):
        thrusts = msg.motorThrusts
        time = FaultyThrustDistributor.get_time()

        for i, (thrust, noise_model, publisher) in enumerate(zip(thrusts, self.noise_models, self.publishers)):
            msg = FloatStamped()
            msg.data = thrust + noise_model(time)
            publisher.publish(msg)

    @staticmethod
    def run():
        rospy.spin()
