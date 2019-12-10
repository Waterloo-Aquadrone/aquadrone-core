import rospy
from aquadrone_msgs.msg import MotorControls

class ThrusterCollectionManager:
    
    def __init__(self, thrusters):
        self.thrusters = thrusters
        self.control_sub = rospy.Subscriber("motor_command", MotorControls, self.apply_thrusts)


    def apply_thrusts(self, msg):
        limit = min(len(self.thrusters), len(msg.motorThrusts))
        for i in range(0, limit):
            self.thrusters[i].command(msg.motorThrusts[i])