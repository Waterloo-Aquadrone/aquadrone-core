import numpy as np
import Gyro_Input as gyro
import Motor_FBD_Linear as fbd
import Motor_PWM_Converter as pwmConverter
import Motor_Controller as controller

while True:
	heading, roll, pitch = gyro.read_euler()
	thrust = np.array([map(pitch, -90, 90, -2, 2), 0, map(roll, -90, 90, -2, 2), 0, 0, map(heading, 90, 270, -2, 2)])
	thrust = fbd.convertThrust(thrust)
	for i in range(0, 6):
		thrust[i] = pwmConverter.getPWMSignalWidth(thrust[i])
	controller.applyThrusts(thrust)


def map(x, xMin, xMax, yMin, yMax):
	return (x - xMin) / (xMax - xMin) * (yMax - yMin) + yMin