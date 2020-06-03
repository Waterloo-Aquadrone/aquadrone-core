import numpy as np


class Vector:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def magnitude(self):
        return np.linalg.norm([x_i for x_i in [self.x, self.y, self.z] if x_i is not None])

    @staticmethod
    def mean(*vectors):
        return sum(vectors) / len(vectors)

    @staticmethod
    def std_mean(*vectors):
        """
        :@return: the component-wise standard deviation of the mean of a set of vectors with the given standard deviations
        """
        return sum(v ** 2 for v in vectors) ** 0.5 / len(vectors)

    @staticmethod
    def from_msg(msg):
        return Vector(msg.x, msg.y, msg.z)

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar):
        return Vector(self.x / scalar, self.y / scalar, self.z / scalar)

    def __pow__(self, exponent):
        return Vector(self.x ** exponent, self.y ** exponent, self.z ** exponent)  # exponentiation is component-wise


class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_msg(msg):
        return Quaternion(msg.w, msg.x, msg.y, msg.z)


class Submarine:
    def __init__(self, position,
                       velocity,
                       orientation_quat,
                       orientation_rpy,
                       angular_velocity):
        self.position = position
        self.velocity = velocity

        self.orientation_quat = orientation_quat
        self.orientation_rpy = orientation_rpy
        
        self.angular_velocity = angular_velocity

        self.position_var = None
        self.direction_var = None
        self.velocity_var = None
        self.angular_velocity_var = None

    def set_uncertainties(self, position, velocity, orientation_quat, orientation_rpy, angular_velocity):
        self.position_var = position
        self.velocity_var = velocity

        self.orientation_quat_var = orientation_quat
        self.orientation_rpy_var = orientation_rpy

        self.angular_velocity_var = angular_velocity


class Gate:
    def __init__(self, top_corners, bottom_corners):
        self.top_corners = top_corners
        self.bottom_corners = bottom_corners
        self.position = Vector.mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])
        self.top_corners_std = None
        self.bottom_corners_std = None
        self.position_std = None

    def set_uncertainties(self, top_corners, bottom_corners):
        self.top_corners_std = top_corners
        self.bottom_corners_std = bottom_corners
        self.position_std = Vector.std_mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])


class DirectionalMarker:
    def __init__(self, raw_points):
        """
        :param raw_points: The 3 points that make up the directional marker in the order [near, middle, far] where the marker is pointing from near to far
        """
        self.raw_points = raw_points
        self.position = raw_points[1]
        self.direction = raw_points[2] - raw_points[1]
        self.postition_std = None
        self.direction_std = None

    def set_uncertainties(self, raw_points):
        self.postition_std = raw_points[1]
        self.direction_std = Vector.std_mean(raw_points[2], raw_points[1])


class Pole:
    def __init__(self, x, y):
        self.postition = Vector(x, y)
        self.position_std = None

    def set_uncertainties(self, x, y):
        self.position_std = Vector(x, y)


class Wall:
    def __init__(self, m, b):
        # y = mx + b
        self.m = m
        self.b = b
        self.m_std = None
        self.b_std = None

    def set_uncertainties(self, m, b):
        self.m_std = m
        self.b_std = b


class CompetitionMap:
    def __init__(self, sub):
        self.sub = sub
        self.gate = None
        self.directional_marker = None
        self.pole = None
        self.walls = []

    def set_gate(self, gate):
        self.gate = gate

    def set_directional_marker(self, directional_marker):
        self.directional_marker = directional_marker

    def set_pole(self, pole):
        self.pole = pole

    def add_wall(self, wall):
        self.walls.append(wall)


class VisionData:
    def __init__(self, labels, data):
        self.data = zip(labels, data)  # list of tuples of class type, object for all the objects that were found
