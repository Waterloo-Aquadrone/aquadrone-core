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
        """
        This function will be called at a regular rate.
        When the state has finished and wants to finalize, it should just ensure that has_completed returns True.
        Do not manually call finalize, or else it will be called twice!
        """
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
        The reason for exiting should be documented and returned in the exit_code function.
        """
        pass

    def exit_code(self):
        """
        This function should only be called once has_completed returns True.
        Indicates the reason that the state has exited.
        The code 0 signifies that the state completed its objective and exited successfully.
        The code -1 signifies that ros was shutdown.
        Other codes should be numbered 1 and higher, and their explanations should be documented.
        """
        return 0
