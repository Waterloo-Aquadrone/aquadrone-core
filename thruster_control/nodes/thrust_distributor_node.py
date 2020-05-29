#!/usr/bin/env python

import rospy

from thruster_control.thruster_configurations import get_thruster_count
from thruster_control.ThrustDistributor import ThrustDistributor


if __name__ == "__main__":
    rospy.init_node('thrust_distributor')

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    num = get_thruster_count(model)

    distributor = ThrustDistributor(num, namespace="aquadrone")
    distributor.run()
