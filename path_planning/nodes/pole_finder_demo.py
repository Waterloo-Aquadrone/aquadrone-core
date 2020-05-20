#!/usr/bin/env python

import rospy

import path_planning.pole_finder_demo_states as States
import path_planning.ros_modules as ROS_Modules

class PoleFinderDemo:
    def __init__(self):

        self.controls = ROS_Modules.ROSControlsModule()
        self.sub_state = ROS_Modules.ROSStateEstimationModule()
        self.sensors = ROS_Modules.ROSSensorDataModule()

        self.state = States.InitialState()
        self.state.initialize(self.t, self.controls, self.sub_state, None, self.sensors)

        self.goal_depth = -3
        # BGR
        red = (80, 80, 255)
        green = (80, 255, 80)
        blue = (255, 10, 10)
        self.goal_color = red

        self.goal_color_transition = {red: green,
                                      green: blue,
                                      blue: red}
        self.low_colors = {red: (0, 0, 40),
                           green: (0, 60, 0),
                           blue: (60, 0, 0)}

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

            if isinstance(self.state, States.InitialState):
                self.switch_states(States.GoToDepthState(self.goal_depth))
                continue

            if isinstance(self.state, States.GoToDepthState):

                if self.state.depth_is_reached():
                    color_high = self.goal_color
                    color_low = self.low_colors[color_high]
                    self.switch_states(States.ColoredPoleFinderState(color_low, color_high))
                    continue


            if isinstance(self.state, States.ColoredPoleFinderState):

                if self.state.have_found_pole():
                    self.switch_states(States.ColoredPoleApproacherState(color_low, color_high))
                    continue


            if isinstance(self.state, States.ColoredPoleApproacherState):

                if self.state.at_pole():
                    self.goal_color = self.goal_color_transition[self.goal_color]
                    self.goal_depth = self.goal_depth + 1
                    self.switch_states(States.GoToDepthState(self.goal_depth))
                    continue

                if self.state.has_lost_pole():
                    color_high = self.goal_color
                    color_low = self.low_colors[color_high]
                    self.switch_states(States.ColoredPoleFinderState(color_low, color_high))
                    continue



if __name__ == "__main__":
    rospy.init_node("pole_finder_demo")
    pfd = PoleFinderDemo()
    pfd.run()
