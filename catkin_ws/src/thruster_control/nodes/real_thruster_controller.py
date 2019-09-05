#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np

from aquadrone_msgs.msg import MotorControls
from thruster_control.thruster_collection_manager import ThrusterCollectionManager
from thruster_control.thruster_interfaces import SimulatedThrusterInterface

import RPi.GPIO as GPIO

class ThrusterCollectionManager:
    
    def __init__(self, gpio):
        self.gpio = gpio

        rospack = rospkg.RosPack()
        config_path = rospack.get_path('thruster_control') + "/config"

        self.pwmData = np.genfromtxt( config_path +  "/Motor PWM Width - Thrust Relation.csv", delimiter=",")
        #microseconds
        self.pwmSignals = pwmData[:, 0]
        #pounds
        self.motorThrusts = pwmData[:, 1]

        self.motorPins = [17, 27, 22, 5, 6, 13]
        self.pwm = [None, None, None, None, None, None]

        self.pwmOffset = 0.3754

        self.sub = None

    def init_gpio(self):
        print("Init GPIO")
        self.gpio.setmode(self.gpio.BCM)

    def init_thrusters(self):
        print("Init Thruster PWM")
        for i in range(0, 6):
            self.gpio.setup(motorPins[i], self.gpio.OUT)
            self.pwm[i] = GPIO.PWM(motorPins[i], 50)
            self.pwm[i].start(7.5 - pwmOffset)

    def init_subscriber(self):
        self.sub = rospy.Subscriber("motor_command", MotorControls, self.apply_thrusts)

    def apply_thrusts(self, msg):
        print("Applying thrusts")
        for i in range(0, len(self.pwm)):
            print(" i=" + str(i))
            thrust = msg.motorThrusts[i]
            print(" T=" + str(i))
            sig = self.thrust_to_pwm(thrust)
            print(" P=" + str(i))
            self.pwm[i].ChangeDutyCycle(pwm)

    def thrust_to_pwm(self, thrust):
        thrustPWM = np.interp(thrust, motorThrusts, pwmSignals)
		thrustPWM = thrustPWM / 200.0 - self.pwmOffset
        return thrustPWM

    def remove_pwm(self):
        for pwn im self.pwm:
            pwm.stop()



if __name__ == "__main__":
    rospy.init_node('real_thruster_controller')

    tm = ThrusterCollectionManager(GPIO)
    tm.init_gpio()
    tm.init_thrusters()
    tm.init_subscriber()

    while not rospy.is_shutdown():
        rospy.sleep(1)

    tm.remove_pwm()
