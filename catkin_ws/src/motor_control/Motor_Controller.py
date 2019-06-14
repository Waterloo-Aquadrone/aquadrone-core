import RPi.GPIO as GPIO
import time
import numpy as np

#in order of placement on prototyping board, starting from the imu
motorPins = [17, 27, 22, 5, 6, 13]
GPIO.setmode(GPIO.BCM)

pwm = [0, 0, 0, 0, 0, 0]
#offset to correct systematic error
pwmOffset = 0.3754

for i in range(0, 6):
	GPIO.setup(motorPins[i], GPIO.OUT)
	pwm[i] = GPIO.PWM(motorPins[i], 50)
	pwm[i].start(7.5 - pwmOffset)

#wait for motors to initialize
time.sleep(7)

#expect numpy array, of size 6
def applyThrusts(motorPWMs):
	for i in range(0, 6):
		pwm[i].ChangeDutyCycle(motorPWMs[i] / 200 - pwmOffset)