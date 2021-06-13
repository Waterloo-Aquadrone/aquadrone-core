import rospy
from aquadrone_sensors.srv import PixelToCoord, PixelToCoordResponse
import pyzed.sl as sl
import numpy as np

class PixelToCoords:
    def __init__(self):
        rospy.Service('pixel_to_coords', PixelToCoord, self.convert_pixel_to_coords)
        self.zed = sl.Camera()
        self.pixel_size = 2e-6
        self.fx = 700.819
        self.fy = 700.819
        self.cx = 665.465
        self.cy = 371.953

    @staticmethod
    def run():
        rospy.spin()

    def convert_from_uvd(self, u, v, d):
        # https://medium.com/yodayoda/from-depth-map-to-point-cloud-7473721d3f
        d *= self.pixel_size
        x_over_z = (self.cx - u) / self.fx
        y_over_z = (self.cy - v) / self.fy
        z = d / np.sqrt(1. + x_over_z ** 2 + y_over_z ** 2)
        x = x_over_z * z
        y = y_over_z * z
        return x, y, z

    def convert_pixel_to_coords(self, pixels):
        # Using depth and pinhole camera
        image = sl.Mat()
        depth_map = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.LEFT)  # Retrieve left image
            self.zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)  # Retrieve depth
        depth_value = depth_map.get_value(pixels[0], pixels[1])
        x, y, z = self.convert_from_uvd(pixels[0], pixels[1], depth_value)
        msg = PixelToCoordResponse()
        msg.three_d_coordinates = [x, y, z]

        # Using a lower resolution
        point_cloud = sl.Mat()
        width = self.zed.get_resolution().width / 2
        height = self.zed.get_resolution().height / 2
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.GPU, width, height)
        point_3D = point_cloud.get_value(pixels[0], pixels[1])

        msg = PixelToCoordResponse()
        msg.three_d_coordinates = point_3D[:3]
        return msg
