from __future__ import annotations
#import rospy
import numpy as numpy
#import geometry_msgs.msg
#from dataclasses import dataclass

#@dataclass
class Cartesian_Point:
    x: float
    y: float
    z: float

    # init
    # initialize from 3 values that are numbers
    def __init__(self, x, y, z):
        try:
            self.x = int(x)
            self.y = int(y)
            self.z = int(z)
        except ValueError:
            print("Cartesian Point Fatal INIT: Not a number")

    # initializer to create from a ROS message
    @classmethod
    def from_ross_message(ros_message) -> Cartesian_Point:
        print("hi from ros")
#    return Cartesian_Point(ros_message.x, ros_message.y, ros_message.z)

    # print definition
    def __str__(self):
        return f"Cartesian_Point(x: {self.x}, y: {self.y}, z: {self.z})"

    # calculate the distance from itself to the target point
    def distance_to(self, point: Cartesian_Point) -> float:
        if type(point) is not Cartesian_Point:
            return -1
        
        cur_point = numpy.array((self.x, self.y, self.z))
        dest_point = numpy.array((point.x, point.y, point.z))

        return numpy.linalg.norm(cur_point - dest_point)

    # calculates the distance for the x-coordinate
    def x_distance(self, point: Cartesian_Point) -> float:
        if type(point) is not Cartesian_Point:
            return -1
        
        return self.x - point.x

    # calculates the distance for the y-coordinate
    def y_distance(self, point: Cartesian_Point) -> float:
        if type(point) is not Cartesian_Point:
            return -1
            
        return self.y - point.y

    # calculates the distance for the z-coordinate
    def z_distance(self, point: Cartesian_Point) -> float:
        if type(point) is not Cartesian_Point:
            return -1
            
        return self.z - point.z

    # returns a Cartesian_Point that is a scaled value of this point
    def scale_point(self, scale: float) -> Cartesian_Point:
        return Cartesian_Point(self.x * scale, self.y * scale, self.z * scale)
