from threading import Timer
import numpy as np
import rospy
import rospkg

# import RPi.GPIO as GPIO
import board
import busio
import adafruit_pca9685

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class RealThruster:
    """
    Objects from this class are responsible for listening to a topic (provided as an argument on construction) that
    publishes the thrust for an individual thruster. The object must send the corresponding PWM signal to the
    appropriate GPIO pin (also provided as an argument on construction).
    """

    def apply_thrust(self, float_stamped):
        """
        Callback function for receiving thrust messages.
        Should have a queue_size of 1 to prioritize the most recent commands.

        :param float_stamped:
        """
        raise NotImplementedError('Cannot instantiate abstract class!')


class T100Thruster(RealThruster):
    PULSE_WIDTH_CALIBRATION_OFFSET = -84  # determined via testing at a frequency of 60 Hz

    def __init__(self, pwm_freq, motor_index, pwm, namespace='aquadrone'):
        self.pwm_freq = pwm_freq
        self.pwm = pwm

        rospack = rospkg.RosPack()
        config_path = rospack.get_path('thruster_control') + "/config/"

        pwmData = np.genfromtxt(config_path + "pwm_thrust_conversion.csv", delimiter=",", names=True)
        self.pwmSignals_us = pwmData['pwm_width']
        self.motorThrusts_lbs = pwmData['thrust_lbs']

        # send 0 thrust signal to initialize the thruster
        initial_thrust = FloatStamped()
        initial_thrust.data = 0
        self.apply_thrust(initial_thrust)

        Timer(7, lambda: rospy.Subscriber("/%s/thrusters/%d/input" % (namespace, motor_index),
                                          FloatStamped, self.apply_thrust, queue_size=1)) \
            .start()

    def apply_thrust(self, float_stamped):
        thrust = float_stamped.data

        # lb to us
        # Manually set to 1500 if thrust is 0 because of ambiguity due to deadband
        pulse_us = 1500 if thrust == 0 else np.interp(thrust, self.motorThrusts_lbs, self.pwmSignals_us)
        pulse_us += T100Thruster.PULSE_WIDTH_CALIBRATION_OFFSET
        pulse_s = pulse_us / 1.0e6
        pulse_perc = pulse_s * self.pwm_freq
        pulse_duty = int(pulse_perc * 0xffff)
        self.pwm.duty_cycle = pulse_duty

        # print("Thrust: %f" % thrust)
        # print("Pulse us: %f" % pulse_us)
        # print("Pulse s: %f" % pulse_s)
        # print("Perc: %f" % pulse_perc)
        # print("Duty: %d" % pulse_duty)


class UUVThruster(RealThruster):
    def __init__(self):
        raise NotImplementedError('UUV Thruster not implemented!')

    def apply_thrust(self, float_stamped):
        raise NotImplementedError('UUV Thruster not implemented!')
