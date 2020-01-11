#!/usr/bin/env python

import rospy

import path_planning.pole_finder_demo_states as States
import path_planning.ros_modules as ROS_Modules

class PoleFinderDemo:
    def __init__(self):

        self.controls = ROS_Modules.ROSControlsModule()
        self.sub_state = ROS_Modules.ROSStateEstimationModule()
        self.sensors = ROS_Modules.ROSSensorDataModule()

        self.state = States.BaseState()
        self.state.initialize(self.t, self.controls, self.sub_state, None, self.sensors)

        self.rate = rospy.Rate(5)

    def switch_states(self, new_state):
        print("Switching from %s to %s..." % (self.state.state_name(), new_state.state_name()))
        self.state.finalize(self.t, self.controls, self.sub_state, None, self.sensors)

        self.state = new_state
        self.state.initialize(self.t, self.controls, self.sub_state, None, self.sensors)

    def t(self):
        return rospy.Time.now().to_sec()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.state.process(self.t, self.controls, self.sub_state, None, self.sensors)

            if isinstance(self.state, States.BaseState):
                self.switch_states(States.GoToDepthState(3))

            if isinstance(self.state, States.GoToDepthState):
                if self.state.depth_is_reached():
                    print("At depth!")



if __name__ == "__main__":
    rospy.init_node("pole_finder_demo")
    pfd = PoleFinderDemo()
    pfd.run()