import rospy
from path_planning.ros_modules import ROSControlsModule, ROSStateEstimationModule, ROSSensorDataModule


def t():
    return rospy.Time.now().to_sec()


def run_state(state, rate):
    controls = ROSControlsModule()
    sub_state = ROSStateEstimationModule()
    sensors = ROSSensorDataModule()

    state.initialize(t(), controls, sub_state, None, sensors)
    while not rospy.is_shutdown():
        rate.sleep()
        state.process(t(), controls, sub_state, None, sensors)
        if state.has_completed():
            state.finalize(t(), controls, sub_state, None, sensors)
            return state.exit_code()

    # This will only occur if ROS is shutdown externally, such as Ctrl+c from the command line
    # This is the only scenario where -1 will be returned as an exit_code
    controls.halt_and_catch_fire()
    return -1


class BaseState:
    """
    Each state should define how it interacts with the depth and orientation control systems, because those are
    persisted across state changes.
    """
    def state_name(self):
        return "base_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        """
        Set up anything that needs initializing
        Run EACH time the state is chosen as the next state
        process(...) will be called with the next available data
        Note the parameters passed into function may be the same as the ones first passed into process.
        """
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        # Regular tick at some rate
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        """
        Clean up anything necessary.
        Note the parameters passed into function may be the same as the last ones passed into process.
        Note this may be called before the state has completed. If this occurs,
        the state should clean up any resources like normal; the exit_code in this case is not important.
        """
        pass

    def has_completed(self):
        """
        When this function returns True, the state should exit.
        Control may be passed to another state if this is part of a state machine.
        The reason for exiting should be documented and returned in the exit_code funciton.

        :return:
        """
        pass

    def exit_code(self):
        """
        Indicates the reason that the state has exited.
        The code 0 signifies that the state completed its objective and exited successfully.
        The code -1 signifies that ros was shutdown.
        Other codes should be numbered 1 and higher, and their explanations should be documented.
        """
        return 0
