import numpy as np
import time
import keyboard
import Motor_FBD_Linear as fbd
import Motor_PWM_Converter as pwmConverter
import Motor_Controller as controller

#wait for motors to set up
time.sleep(7)

while True:
	x = 0
	z = 0
	yaw = 0
	
	control = input("Command:")
	if control == "w":
		x = 1
	elif control == "s":
		x = -1
	elif control == "a":
		yaw = 1
	elif control == "d":
		yaw = -1
	elif control == "i":
		z = 1
	elif contorl == "k":
		z = -1
	
	thrust = np.array([x, 0, z, 0, 0, yaw])
	thrust = fbd.convertThrusts(thrust)
	for i in range(0, 6):
		thrust[i] = pwmConverter.getPWMSignalWidth(thrust[i])
	controller.applyThrusts(thrust)
	print("x: " + str(x) + ", z:" + str(z) + ", yaw:" + str(yaw))
	time.sleep(0.05)