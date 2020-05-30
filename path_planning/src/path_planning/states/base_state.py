import rospy
import path_planning.ros_modules as ROS_Modules


def t():
    return rospy.Time.now().to_sec()


def run_state(state, rate=rospy.Rate(5)):
    controls = ROS_Modules.ROSControlsModule()
    sub_state = ROS_Modules.ROSStateEstimationModule()
    sensors = ROS_Modules.ROSSensorDataModule()

    state.initialize(t(), controls, sub_state, None, sensors)
    while not rospy.is_shutdown():
        rate.sleep()
        state.process(t(), controls, sub_state, None, sensors)
        if state.has_completed():
            state.finalize(t(), controls, sub_state, None, sensors)
            return state.exit_code()
    return -1


class BaseState:
    """
    Each state should define how it interacts with the depth and orientation control systems, because those are
    persisted across state changes.
    """
    def state_name(self):
        return "base_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        # Set up anything that needs initializing
        # Run EACH time the state is chosen as the next state
        # process(...) will be called with the next available data
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        # Regular tick at some rate
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        # Clean up anything necessary
        pass

    def has_completed(self):
        pass

    def exit_code(self):
        """
        The code -1 signifies that ros was shutdown.

        :return:
        """
        return 0
