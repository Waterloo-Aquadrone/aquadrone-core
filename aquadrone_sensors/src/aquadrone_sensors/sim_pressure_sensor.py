import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import FluidPressure


class SimPressureSensor:
    WATER_SPECIFIC_GRAVITY = 1000 * 9.81  # Pa/m
    ATMOSPHERIC_PRESSURE = 101325  # Pa

    def __init__(self, rate=None, sub_name='aquadrone'):
        self.rate = rate if rate is not None else rospy.rate(10)
        self.sub_name = sub_name
        self.object_pos_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.get_obj_pos)
        self.pub = rospy.Publisher("/aquadrone/sensors/pressure", FluidPressure, queue_size=1)
        self.measurement_z = 0

    def get_obj_pos(self, data):
        for name, pose in zip(data.name, data.pose):
            if name == self.sub_name:
                self.measurement_z = pose.z

    def publish_message(self):
        depth = -self.measurement_z
        gauge_pressure = self.WATER_SPECIFIC_GRAVITY * depth
        absolute_pressure = gauge_pressure + self.ATMOSPHERIC_PRESSURE

        msg = FluidPressure()
        msg.fluid_pressure = absolute_pressure
        self.pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

            self.publish_message()
