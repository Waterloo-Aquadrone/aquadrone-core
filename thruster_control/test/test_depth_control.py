#!/usr/bin/env python

import rospy
import rostest
import unittest
from mock import MagicMock

import thruster_control
from thruster_control.depth_pid_controller import DepthPIDController

from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import Wrench
from aquadrone_msgs.msg import SubState


class TestDepthPIDController(unittest.TestCase):

    def test_contruction(self):
        pid = DepthPIDController()

    def test_publishes_wrench_with_u(self):
        # Want to test that the object actually publishes the correct wrench
        # Note: All this documentation is for learning purposes.
        #       Don't add this much to every test

        # Init
        pid = DepthPIDController()

        # Replace the function of an object with a MagicMock object
        #  this tracks calls made to that function
        #  so we can see how the .publish function was called
        pid.w_pub.publish = MagicMock()

        # Define what we expect the function to be called with
        exp = Wrench()
        exp.force.z = 1.5

        # We expect this command to lead to the wrench being published
        pid.publish_wrench(1.5)

        # Now see what it was actually called with
        pid.w_pub.publish.assert_called_with(exp)
        # This would fail for ex, if:
        #  it published a Vector3 (3d force)
        #  if the negative of the force value was used
        #  if the force was assigned to the x or y component instead

    def test_updates_current_depth(self):
        pid = DepthPIDController()
        pid.depth = 0 # Make sure we have init condition

        # Set up message
        in_depth = 5.0
        msg = SubState()
        msg.position.z = -in_depth

        # Run code
        pid.state_cb(msg)

        # No need to mock, since the method updates a variable we can access
        self.assertAlmostEqual(in_depth, pid.depth)

        


if __name__ == '__main__':
    rospy.init_node('test_depth_control')
    rostest.rosrun('thruster_control', 'test_depth_pid_control', TestDepthPIDController)
