import numpy as np

def get_wrench_to_thrusts_lb_in():

    xLength = 36.0
    yLength = 12.0

    #correction factor for yaw induced when attempting to roll
    rollCorr = np.sqrt(2) * xLength / yLength

    #all of these are normalized to produce a net force of 1 pound
    relativeXThrusts = np.array([0, 0, 1, 1, 0, 0]) / 2.0
    relativeYThrusts = np.array([1, -1, 0, 0, -1, 1]) / (2.0 * np.sqrt(2))
    relativeZThrusts = np.array([-1, -1, 0, 0, -1, -1]) / (2.0 * np.sqrt(2))

    #all of these are normalized to produce a net torque of 1 pound-inch
    relativeRollThrusts = np.array([-1, 1, -rollCorr, rollCorr, -1, 1]) / (yLength * np.sqrt(2))
    relativePitchThrusts = np.array([1, 1, 0, 0, -1, -1]) / (xLength * np.sqrt(2))
    relativeYawThrusts = np.array([0, 0, -1, 1, 0, 0]) / yLength

    thrustMatrix = np.column_stack((relativeXThrusts, relativeYThrusts, relativeZThrusts, 
                relativeRollThrusts, relativePitchThrusts, relativeYawThrusts))

    return thrustMatrix