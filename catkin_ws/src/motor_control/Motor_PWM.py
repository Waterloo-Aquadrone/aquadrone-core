import numpy as np

data = np.genfromtxt("Motor PWM Width - Thrust Relation.csv", delimiter=",")
#microseconds
pwmSignals = data[:, 0]
#pounds
motorThrusts = data[:, 1]

def getPWMSignalWidth(thrust):
	return np.interp(thrust, motorThrusts, pwmSignals)