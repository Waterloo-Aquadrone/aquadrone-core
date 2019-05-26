#!/usr/bin/env python

import time
import rospy

from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import ApplyBodyWrench


class FakeThrusterInterface:
    def __init__(self):
        rospy.init_node("fake_thruster_interface", anonymous=False)
        rospy.wait_for_service('/gazebo/apply_body_wrench')

        self.wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        self.T1 = 1
        self.T2 = -1

        self.side_dy = 0.3

        self.dt = 0.1
        self.duration = 0.05

    def calc_x_thrust(self):
        return self.T1 + self.T2

    def calc_z_moment(self):
        return self.side_dy * (self.T1 - self.T2)

    def apply_thrust(self):
        wrench = Wrench()
        wrench.force.x = self.calc_x_thrust()
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = self.calc_z_moment()

        try:
            self.wrench_service(body_name = "sub::base_link",
                        reference_frame = "sub::base_link",
                        wrench = wrench,
                        duration = rospy.Duration(self.duration))
        except rospy.ServiceException as e:
            pass
            #print e

    def run(self):
        while not rospy.is_shutdown():
            self.apply_thrust()
            rospy.sleep(self.dt)

if __name__ == "__main__":
    FTI = FakeThrusterInterface()
    FTI.run()





    