import numpy as np
import time
import Gyro_Input as gyro
import Motor_FBD_Linear as fbd
import Motor_PWM_Converter as pwmConverter
import Motor_Controller as controller

#wait for motors to set up
time.sleep(7)

#let user adjust heading to 180
for i in range(0, 40):
	heading, roll, pitch = gyro.read_euler()
	print("heading:" + str(heading))
	time.sleep(0.25)

while True:
	heading, roll, pitch = gyro.read_euler()
	x = pitch / 45
	z = roll / 45
	yaw = (heading - 180) / 45 / 4
	thrust = np.array([x, 0, z, 0, 0, yaw])
	thrust = fbd.convertThrusts(thrust)
	for i in range(0, 6):
		thrust[i] = pwmConverter.getPWMSignalWidth(thrust[i])
	controller.applyThrusts(thrust)
	print("x: " + str(x) + ",z:" + str(z) + ",yaw:" + str(yaw))
	time.sleep(0.05)
