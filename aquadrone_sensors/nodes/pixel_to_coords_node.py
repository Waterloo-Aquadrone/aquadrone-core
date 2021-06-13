import rospy
from aquadrone_sensors.pixel_to_coords import PixelToCoords


if __name__ == "main":
    rospy.init_node('pixel_to_coords', log_level=rospy.DEBUG)

    pixel_to_coords = PixelToCoords()
    pixel_to_coords.run()
