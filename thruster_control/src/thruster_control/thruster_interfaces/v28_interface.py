#import RPi.GPIO as GPIO

import rospy

import board
import busio
import adafruit_pca9685

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped



class V28ThrusterInterface:
    def __init__(self):
        #self.gpio = gpio
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)

        self.motorPins = [1, 2, 3, 4, 5, 6, 7, 8]
        self.pwm = [None, None, None, None, None, None, None, None]

        self.frequency = 400


    def init_gpio(self):
        print("Init PCA")
        #self.gpio.setmode(self.gpio.BCiM)
        self.pca.frequency = self.frequency

    def init_thrusters(self, thruster_specs):
        print("Init Thruster PWM")
        for i in range(0, 8):
            self.pwm[i] = self.pca.channels[self.motorPins[i]]
            self.pwm[i].duty_cycle = thruster_specs[i].get_zero_signal()
            #self.gpio.setup(self.motorPins[i], self.gpio.OUT)
            #self.pwm[i] = GPIO.PWM(self.motorPins[i], 50)
            #self.pwm[i].start(7.5 - pwmOffset)

    def command(self, idx, sig):
        self.pwm[idx].duty_cycle = sig

    def get_frequency(self):
        return self.frequency
