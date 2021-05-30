import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import FluidPressure
from aquadrone_math_utils.quaternion import Quaternion

class SimPressureSensor:
    WATER_SPECIFIC_GRAVITY = 1000 * 9.81  # Pa/m
    ATMOSPHERIC_PRESSURE = 101325  # Pa
    VARIANCE = (0.1 * WATER_SPECIFIC_GRAVITY) ** 2  # Pa^2

    def __init__(self, rate=None, sub_name='aquadrone'):
        self.rate = rate if rate is not None else rospy.Rate(10)
        self.sub_name = sub_name
        self.object_pos_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.get_obj_pos)
        self.pub = rospy.Publisher("/aquadrone/sensors/pressure", FluidPressure, queue_size=1)
        self.measurement_z = 0
        pressure_offset_dict = rospy.get_param("/submarine/pressure_sensor_offset")
        self.pressure_offset = [pressure_offset_dict["x"], pressure_offset_dict["y"], pressure_offset_dict["z"]]

    def get_obj_pos(self, data):
        for name, pose in zip(data.name, data.pose):
            if name == self.sub_name:
                quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                quad_orientation = Quaternion.from_array(quat)
                rotated_offset = quad_orientation.rotate(self.pressure_offset)
                self.measurement_z = pose.position.z + rotated_offset[2]

    def publish_message(self):
        depth = -self.measurement_z
        gauge_pressure = self.WATER_SPECIFIC_GRAVITY * depth
        absolute_pressure = gauge_pressure + self.ATMOSPHERIC_PRESSURE

        msg = FluidPressure()
        msg.fluid_pressure = absolute_pressure
        msg.variance = self.VARIANCE
        self.pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

            self.publish_message()
