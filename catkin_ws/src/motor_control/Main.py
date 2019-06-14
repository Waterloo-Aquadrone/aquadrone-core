import numpy as np
import time
import Gyro_Input as gyro
import Motor_FBD_Linear as fbd
import Motor_PWM_Converter as pwmConverter
import Motor_Controller as controller

time.sleep(10)

while True:
	heading, roll, pitch = gyro.read_euler()
	x = pitch / 45
	z = roll / 45
	yaw = (heading - 180) / 45
	thrust = np.array([x, 0, z, 0, 0, yaw])
	thrust = fbd.convertThrust(thrust)
	for i in range(0, 6):
		thrust[i] = pwmConverter.getPWMSignalWidth(thrust[i])
	controller.applyThrusts(thrust)