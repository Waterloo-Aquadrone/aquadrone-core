#!/usr/bin/env python

import rospy

from path_planning.base_state import InitialState
import path_planning.diagnostic_states as Diagnostics
import path_planning.basic_control_states as BasicControls

import path_planning.ros_modules as ROS_Modules

class PACDepthControlTest:
    def __init__(self):

        self.controls = ROS_Modules.ROSControlsModule()
        self.sub_state = ROS_Modules.ROSStateEstimationModule()
        self.sensors = ROS_Modules.ROSSensorDataModule()

        self.state = InitialState()
        self.state.initialize(self.t, self.controls, self.sub_state, None, self.sensors)

        self.depth_state_num = 0

        self.rate = rospy.Rate(5)


    def switch_states(self, new_state):
        print("Switching from %s to %s..." % (self.state.state_name(), new_state.state_name()))
        self.state.finalize(self.t(), self.controls, self.sub_state, None, self.sensors)

        self.state = new_state
        self.state.initialize(self.t(), self.controls, self.sub_state, None, self.sensors)

    def t(self):
        return rospy.Time.now().to_sec()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.state.process(self.t(), self.controls, self.sub_state, None, self.sensors)

            if isinstance(self.state, InitialState):
                self.switch_states(Diagnostics.CheckThrustersState(time_each=3))
                continue

            if isinstance(self.state, Diagnostics.CheckThrustersState):

                if self.state.all_thrusters_tested():
                    self.switch_states(BasicControls.WaitForSubmergenceState())
                    continue

            if isinstance(self.state, BasicControls.WaitForSubmergenceState):

                if self.state.has_submerged():
                    self.depth_state_num = 1
                    self.switch_states(BasicControls.GoToDepthState(d_goal=1, t_required=15))
                    continue

            if isinstance(self.state, BasicControls.GoToDepthState) and self.depth_state_num == 1:

                if self.state.depth_is_reached():
                    self.depth_state_num = 2

                    self.controls.set_yaw_goal(3.0) # Just to test it out

                    self.switch_states(BasicControls.GoToDepthState(d_goal=2, t_required=15))
                    continue

            if isinstance(self.state, BasicControls.GoToDepthState) and self.depth_state_num == 2:

                if self.state.depth_is_reached():

                    self.controls.planar_move_command(Fx=2.0, Fy=0, Tz=0) # Just to test it out
                    rospy.sleep(3.0)
                    self.controls.planar_move_command(Fx=0, Fy=0, Tz=0)

                    self.switch_states(BasicControls.GoToSurfaceState())
                    continue

            if isinstance(self.state, BasicControls.GoToSurfaceState):

                if self.state.has_surfaced():
                    self.switch_states(BasicControls.TurnOffForRetrievalState())
                    continue

            if isinstance(self.state, BasicControls.TurnOffForRetrievalState):

                if self.state.has_turned_off():
                    # No switch; End of test
                    continue



if __name__ == "__main__":
    rospy.init_node("pac_depth_control_test")
    test = PACDepthControlTest()
    test.run()