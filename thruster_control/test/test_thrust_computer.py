#!/usr/bin/env python3

# import rospy
# import rostest
import unittest
# from mock import MagicMock
from time import time

from thruster_control.thrust_computer.thrust_computer import ThrustComputer


class TestThrustComputer(unittest.TestCase):
    # def test_contruction(self):
    #     c = ThrustComputer(None, None)
    
    def test_thrusts_optimization(self):
        thrust_one = [15,15,15,15,15,15,15,15]
        thrust_two = [20,1,1,1,1,1,1,1]
        thrust_three = [1.9,1,1,1,1,1,1,1]

        thrusts = [thrust_one, thrust_two, thrust_three]
        starttime = time()
        final_thrusts = ThrustComputer.optimize_thursts(thrusts)

        print(final_thrusts)
        endtime = time()
        print(endtime - starttime)

    # create a seperate function here

    # three vectors of 8 thrusts
    # sends this

    #send back one vector of thrusts

    # also make seperate function in control loop

if __name__ == '__main__':
    # rospy.init_node('test_thrust_computer')
    # rostest.rosrun('thruster_control', 'test_thrust_computer', TestThrustComputer)
    unittest.main()