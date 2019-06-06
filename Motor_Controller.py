import numpy as np

#assumptions:
#	see diagrams for coordinate system and motor numbering
#	motors 0, 1, 4, and 5 are mounted at 45 degrees
#	positive thrust aims towards the negative z and positive y direction for motors 0 and 5
#	positive thrust aims towards the negative z and negative y direction for motors 1 and 4
#	positive thrust aims directly in the positive x direction for motors 2 and 3
#	positive pitch, roll, and yaw are determined via the corresponding axis using the right hand rule

#inches
xLength = 36
ylength = 12

#pounds
motorThrusts = np.array([0, 0, 0, 0, 0, 0])

#correction factor for yaw induced when attempting to roll
rollCorr = np.sqrt(2) * xLength / ylength

#all of these are normalized to produce a net force of 1 pound
relativeXThrusts = np.array([0, 0, 1, 1, 0, 0]) / 2
relativeYThrusts = np.array([1, -1, 0, 0, -1, 1]) / (2 * np.sqrt(2))
relativeZThrusts = np.array([-1, -1, 0, 0, -1, -1]) / (2 * np.sqrt(2))

#all of these are normalized to produce a net torque of 1 pound-inch
relativeRollThrusts = np.array([-1, 1, -rollCorr, rollCorr, -1, 1]) / (ylength * np.sqrt(2))
relativePitchThrusts = np.array([1, 1, 0, 0, -1, -1]) / (xLength * np.sqrt(2))
relativeYawThrusts = np.array([0, 0, -1, 1, 0, 0]) / yLength

thrustMatrix = np.column_stack(relativeXThrusts, relativeYThrusts, relativeZThrusts, 
			relativeRollThrusts, relativePitchThrusts, relativeYawThrusts)

#input is [xThrust, yThrust, zThrust, rollTorque, pitchTorque, yawTorque]
#all inputs in pounds or pound-inches respectively
#the thrusts/torques are applied in addition to whatever the current thruster values are
def applyThrusts(vector):
	motorThrusts += thrustMatrix * vector