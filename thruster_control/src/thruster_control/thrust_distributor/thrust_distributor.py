import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_srvs.srv import Trigger, TriggerResponse
from aquadrone_msgs.msg import MotorControls


class ThrustDistributor:
    """
    Receives commands with a list of thrusts for each thruster, and distributes them in separate topics
    Works for both simulated and real thrusters.
    """
    def __init__(self, thruster_count, namespace='aquadrone', rate=None):
        if rate is None:
            # should be checking fairly fast because we want to kill thrust immediately
            self.rate = rospy.Rate(10)

        self.thruster_count = thruster_count
        rospy.Subscriber('motor_command', MotorControls, callback=self.process_thrust_command)

        self.publishers = []
        for i in range(0, thruster_count):
            pub = rospy.Publisher("/%s/thrusters/%d/input" % (namespace, i), FloatStamped, queue_size=1)
            self.publishers.append(pub)

        self.terminate = False
        rospy.Service('halt_and_catch_fire', Trigger, self.halt_and_catch_fire)

    def process_thrust_command(self, msg):
        thrusts = msg.motorThrusts

        for i, thrust in enumerate(thrusts):
            msg = FloatStamped()
            msg.data = thrust
            self.publishers[i].publish(msg)

    def halt_and_catch_fire(self, msg=None):
        self.terminate = True
        return TriggerResponse(success=True, message="Stopping Thrusters")

    def run(self):
        while not rospy.is_shutdown() and not self.terminate:
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

        # send command to zero out all thrusts
        msg = MotorControls()  # defaults to all zeros
        self.process_thrust_command(msg)
