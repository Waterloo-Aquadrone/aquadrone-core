import numpy as np
import time
import Motor_FBD_Linear as fbd
import Motor_PWM_Converter as pwmConverter
import Motor_Controller as controller

#wait for motors to set up
time.sleep(7)

while True:
	x = 0
	z = 0
	yaw = 0
	
	ctrl = raw_input("Command:")
	if ctrl == "w":
		x = 1
	elif ctrl == "s":
		x = -1
	elif ctrl == "a":
		yaw = 1
	elif ctrl == "d":
		yaw = -1
	elif ctrl == "i":
		z = 1
	elif ctrl == "k":
		z = -1
	
	thrust = np.array([x, 0, z, 0, 0, yaw])
	thrust = fbd.convertThrusts(thrust)
	for i in range(0, 6):
		thrust[i] = pwmConverter.getPWMSignalWidth(thrust[i])
	controller.applyThrusts(thrust)
	print("x: " + str(x) + ", z:" + str(z) + ", yaw:" + str(yaw))
	time.sleep(0.05)
