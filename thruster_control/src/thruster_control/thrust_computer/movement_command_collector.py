import rospy
from geometry_msgs.msg import Wrench
from aquadrone_math_utils.ros_utils import wrench_to_np


class CommandSubscriber:
    # Should listen to a wrench command from some source topic
    # Track the latest command, as well as the last time it was received
    def __init__(self, topic):
        self.cmd = Wrench()
        self.time = rospy.Time()
        rospy.Subscriber(topic, Wrench, self.callback)

    def callback(self, wrench):
        self.cmd = wrench
        self.time = rospy.Time()

    def get_cmd(self):
        return self.cmd, (rospy.Time() - self.time).to_sec()


class MovementCommandCollector:
    """
    This class can be used to listen to a list of topics for motor thrust commands (in the form of a Wrench).
    The get_recent_thrusts method can be used to get the sum of the most recent Wrenches from each of the topics.
    A command timeout can be specified on construction to disregard and topics that have not had a message published in
    for the specified amount of time.
    """
    def __init__(self, cmd_timeout=0.5):
        # Set up command sources
        # First added is highest priority
        self.sources = [CommandSubscriber(topic) for topic in
                        ['/stability_command', '/depth_command', '/movement_command']]
        self.cmd_timeout = cmd_timeout

    def get_recent_wrenches(self):
        """
        Returns the desired Wrench based on the sum of the Wrenches from the sources.
        Sources may be ignored if the cmd_timeout has expired.
        """
        return [self.get_source_command(source) for source in self.sources]

    def get_source_command(self, source):
        # Return the latest command from the source
        # Set to 0 if not received for some time
        cmd, dt = source.get_cmd()
        cmd = wrench_to_np(cmd)
        if dt > self.cmd_timeout:
            cmd = cmd * 0.0
        return cmd
