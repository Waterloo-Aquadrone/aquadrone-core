#!/usr/bin/env python

import rospy

from thruster_control.thruster_configurations import V1Configuration, V2Configuration, V28Configuration
from thruster_control.movement_command_collector import MovementCommandCollector
from thruster_control.thruster_interfaces.sim_interface import SimulatedThrusterInterface
from thruster_control.thruster_types import UUVSimThruster
from thruster_control.thruster_control_manager import SimThrusterController


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

    interface = SimulatedThrusterInterface("aquadrone", num)
    interface.initialize()

    th_spec = UUVSimThruster()
    th_spec.initialize()
    specs = [th_spec for i in range(0, num)]

    do_control_loop = rospy.get_param("control_loop", True)
    stc = SimThrusterController(config, mcc, interface, specs, do_control_loop=do_control_loop)
    stc.run()
