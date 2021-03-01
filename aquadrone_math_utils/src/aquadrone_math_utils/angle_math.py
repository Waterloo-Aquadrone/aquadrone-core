import numpy as np


def normalize_angle(angle, radians=True):
    """
    Normalizes the given angle to the range [0, 2 * pi) for radians and [0, 360) for degrees.

    :param angle:
    :param radians:
    :return:
    """
    revolution = 2 * np.pi if radians else 360
    while angle > revolution:
        angle -= revolution
    while angle < 0:
        angle += revolution
    return angle


def abs_angle_difference(a1, a2, radians=True):
    """
    Returns the absolute value of the difference between the two given angles.
    The result will be a number in the range [0, 2 * pi] for radians, and [0, 360) for degrees.

    :param a1: The first angle.
    :param a2: The second angle.
    :param radians: True if the angles are in radians, False if they are in degrees.
    :return: The absolute value of the difference between the two given angles.
    """
    revolution = 2 * np.pi if radians else 360

    diff = normalize_angle(a1 - a2, radians=radians)

    # if greater than pi, then going the other direction is shorter
    if diff > revolution / 2:
        diff = revolution - diff
    return diff
