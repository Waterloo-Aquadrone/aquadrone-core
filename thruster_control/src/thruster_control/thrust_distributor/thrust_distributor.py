import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from aquadrone_msgs.msg import MotorControls


class ThrustDistributor:
    """
    Receives commands with a list of thrusts for each thruster, and distributes them in separate topics
    Works for both simulated and real thrusters.
    """
    def __init__(self, thruster_count, namespace='aquadrone'):
        self.thruster_count = thruster_count
        rospy.Subscriber('motor_command', MotorControls, callback=self.process_thrust_command)

        self.publishers = []
        for i in range(0, thruster_count):
            pub = rospy.Publisher("/%s/thrusters/%d/input" % (namespace, i), FloatStamped, queue_size=1)
            self.publishers.append(pub)

    def process_thrust_command(self, msg):
        thrusts = msg.motorThrusts

        for i, thrust in enumerate(thrusts):
            msg = FloatStamped()
            msg.data = thrust
            self.publishers[i].publish(msg)

    @staticmethod
    def run():
        rospy.spin()
