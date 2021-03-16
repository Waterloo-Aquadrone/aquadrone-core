#!/usr/bin/env python3

import rospy
import rostest
import unittest
from mock import MagicMock
from scipy.spatial.transform import Rotation
import numpy as np
import aquadrone_math_utils.orientation_math as OMath

class TestOMath(unittest.TestCase):

    def test_omath(self):
        OMath.Roll(0)
        OMath.Pitch(0)
        OMath.Yaw(0)

    def check_omath_rotation(self, quat_vec):
        rotation_quat_to_euler = Rotation.from_quat(quat_vec[1:] + [quat_vec[0]]).as_euler('ZYX')
        quat_vec = np.array(quat_vec)
        omath_quat_to_euler = np.array(OMath.quaternion_to_euler(quat_vec/np.sum(quat_vec*quat_vec)**0.5), dtype=float)
        omath_equal_rotation = np.all(np.isclose(omath_quat_to_euler,rotation_quat_to_euler))
        self.assertTrue(omath_equal_rotation)

    def test_quat_to_euler(self):
        self.check_omath_rotation([0.1, 0.2, 0.3, 0.4])
        self.check_omath_rotation([0.1, -0.2, 0.15, 0.1])
        self.check_omath_rotation([0.5, 0.5, 0.5, -0.5])
        self.check_omath_rotation([0.5, -0.5, 0.5, 0.5])


if __name__ == '__main__':
    rospy.init_node('test_o_math')
    rostest.rosrun('aquadrone_math_utils', 'test_orientation_math', TestOMath)
