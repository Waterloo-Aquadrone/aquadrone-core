import numpy as np

data = np.genfromtxt("Motor PWM Width - Thrust Relation.csv", delimiter=",")
#microseconds
pwmSignals = data[:, 0]
#pounds
motorThrusts = data[:, 1]

#expect thrust in pounds
def getPWMSignalWidth(thrust):
	return np.interp(thrust, motorThrusts, pwmSignals)