from dataclasses import dataclass
from points_helper import Cartesian_Point

def get_next_point(points: [Cartesian_Point]) -> Cartesian_Point:
    # find  the middle x, y, z point
    x_dest = sum([point.x for point in points]) / len(points)
    y_dest = sum([point.y for point in points]) / len(points)
    z_dest = sum([point.z for point in points]) / len(points)
    
    return Cartesian_Point(x_dest, y_dest, z_dest)

def get_next_move():
    
    # get coordinate of the destination by posting ros message to camera/target aquisition

    # example of points to get back
    point1 = Cartesian_Point(0.0, 0.0, 0.0)
    point2 = Cartesian_Point(0.0, 4.0, 0.0)
    point3 = Cartesian_Point(4.0, 0.0, 0.0)
    point4 = Cartesian_Point(4.0, 4.0, 0.0)

    next_point = get_next_point([point1, point2, point3, point4])

    # determine next move to make, prioritize z and then x/y
    if (next_point.z > 1):
        next_move = Cartesian_Point(0.0, 0.0, 1.0)
    else:
        if (next_point.x > 1):
            next_move = Cartesian_Point(1.0, 0.0, 0.0)
        elif (next_point.y > 1):
            next_move = Cartesian_Point(0.0, 1.0, 0.0)
        else:
            next_move = Cartesian_Point(0.0, 0.0, 0.0)

    return next_move
    # post next move to local path finding


print(get_next_move())
