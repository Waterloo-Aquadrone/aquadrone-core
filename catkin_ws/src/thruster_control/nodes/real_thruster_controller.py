#!/usr/bin/env python2

import rospy
import rospkg
import numpy as np

from aquadrone_msgs.msg import MotorControls
from thruster_control.thruster_collection_manager import ThrusterCollectionManager
from thruster_control.thruster_interfaces import SimulatedThrusterInterface

import board
import busio
import adafruit_pca9685

#import RPi.GPIO as GPIO

class ThrusterCollectionManager:
    
    def __init__(self):
        #self.gpio = gpio
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)

        rospack = rospkg.RosPack()
        config_path = rospack.get_path('thruster_control') + "/config"

        self.pwmData = np.genfromtxt( config_path +  "/pwm_thrust_conversion.csv", delimiter=",")
        #microseconds
        self.pwmSignals = self.pwmData[:, 0]
        #pounds
        self.motorThrusts = self.pwmData[:, 1]

        self.motorPins = [0, 1, 2, 3, 4, 5]
        self.pwm = [None, None, None, None, None, None]

        self.pwmOffset = 0.3754

        self.sub = None

    def init_gpio(self):
        print("Init PCA")
        #self.gpio.setmode(self.gpio.BCiM)
        self.pca.frequency = 60

    def init_thrusters(self):
        print("Init Thruster PWM")
        for i in range(0, 6):
            self.pwm[i] = self.pca.channels[self.motorPins[i]]
            self.pwm[i].duty_cycle = 0
            #self.gpio.setup(self.motorPins[i], self.gpio.OUT)
            #self.pwm[i] = GPIO.PWM(self.motorPins[i], 50)
            #self.pwm[i].start(7.5 - pwmOffset)

    def init_subscriber(self):
        self.sub = rospy.Subscriber("motor_command", MotorControls, self.apply_thrusts)

    def apply_thrusts(self, msg):
        print("Applying thrusts")
        for i in range(0, len(self.pwm)):
            print(" i=" + str(i))
            thrust = msg.motorThrusts[i]
            print(" T=" + str(thrust))
            sig = self.thrust_to_pwm(thrust)
            print(" sig=" + str(sig))
            self.pwm[i].duty_cycle = int(sig)

    def thrust_to_pwm(self, thrust):
        thrustPWM = np.interp(thrust, self.motorThrusts, self.pwmSignals)
        thrustPWM = thrustPWM / 200.0 - self.pwmOffset
        return thrustPWM

    def remove_pwm(self):
        for pwn in self.pwm:
            #pwm.stop()
            pwm.duty_cycle=0



if __name__ == "__main__":
    rospy.init_node('real_thruster_controller')

    tm = ThrusterCollectionManager()
    tm.init_gpio()
    tm.init_thrusters()
    tm.init_subscriber()

    while not rospy.is_shutdown():
        rospy.sleep(1)

    tm.remove_pwm()
