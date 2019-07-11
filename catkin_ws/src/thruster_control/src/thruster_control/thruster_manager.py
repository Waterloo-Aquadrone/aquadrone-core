import rospy
from aquadrone_msgs.msg import MotorControls

class ThrusterManager:
    num_thrusters = 6
    
    def __init__(self, thrusters):
        self.thrusters = thrusters
        self.control_sub = rospy.Subscriber("motor_command", MotorControls, self.apply_thrusts)


    def apply_thrusts(self, msg):
        for i in range(0, self.num_thrusters):
            self.thrusters[i].command(msg.motorThrusts[i])