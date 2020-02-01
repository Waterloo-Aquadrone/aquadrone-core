import numpy as np
import math
import rospy
import rospkg


a2 = math.pi / 2.0
a4 = math.pi / 4.0
in2m = 2.54 * 0.01

class ThrusterType:
    def __init__(self):
        raise NotImplementedError
    def initialize(self):
        raise NotImplementedError
    def thrust_to_signal(self):
        raise NotImplementedError
    def get_zero_signal(self):
        raise NotImplementedError


class BlueRoboticsT100(ThrusterType):
    def __init__(self):
        self.pwmOffset = 0.3754
        self.pwmData = None
        self.pwmSignals_us = None
        self.motorThrusts_lbs = None

    def initialize(self):
        rospack = rospkg.RosPack()
        config_path = rospack.get_path('thruster_control') + "/config"
        self.pwmData = np.genfromtxt( config_path +  "/pwm_thrust_conversion.csv", delimiter=",")
        self.pwmSignals_us = self.pwmData[:, 0]
        self.motorThrusts_lbs = self.pwmData[:, 1]

    def thrust_to_signal(self, thrust):
        # lb to us
        thrustPWM = np.interp(thrust, self.motorThrusts_lbs, self.pwmSignals_us)
        thrustPWM = thrustPWM / 200.0 - self.pwmOffset
        return thrustPWM

    def get_zero_signal(self):
        return self.thrust_to_signal(0)

class UUVSimThruster(ThrusterType):
    def __init__(self):
        pass
    def initialize(self):
        pass
    def thrust_to_signal(self, thrust):
        return thrust
    def get_zero_signal(self):
        return 0
