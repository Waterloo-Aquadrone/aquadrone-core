from dataclasses import dataclass

@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

def get_next_move(points: [Point]):
    # find  the middle x, y, z point
    x_dest = sum([point.x for point in points]) / len(points)
    y_dest = sum([point.y for point in points]) / len(points)
    z_dest = sum([point.z for point in points]) / len(points)

    #
    
    
    return [x_dest, y_dest, z_dest]




point1 = Point(0.0, 0.5, 0.1)
print(get_next_move([point1, point1, point1, point1]))
