#!/usr/bin/env python

import rospy

from thruster_control.thruster_configurations import V1Configuration, V2Configuration, V28Configuration
from thruster_control.movement_command_collector import MovementCommandCollector
from thruster_control.thruster_interfaces.v28_interface import V28ThrusterInterface
from thruster_control.thruster_types import BlueRoboticsT100
from thruster_control.thruster_control_manager import RealThrusterController


if __name__ == "__main__":
    rospy.init_node('sim_thruster_controller')

    mcc = MovementCommandCollector()

    # Assume V28 by default
    model = rospy.get_param("model", "v28")
    num = 8

    if model == "v1":
        config = V1Configuration()
        num = 6
    elif model == "v2":
        config = V1Configuration()
        num = 6
    elif model == "v28":
        config = V28Configuration()
        num = 8
    else:
        print("Error: unknown model for controls: %s" % str(model))

    config.initialize()

    th_spec = BlueRoboticsT100()
    th_spec.initialize()
    specs = [th_spec for i in range(0, 8)]

    interface = V28ThrusterInterface()
    interface.init_gpio()
    interface.init_thrusters(specs)

    

    stc = RealThrusterController(config, mcc, interface, specs)
    stc.run()
   