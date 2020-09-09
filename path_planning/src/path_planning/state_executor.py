import rospy
from .ros_modules import ROSControlsModule, ROSStateEstimationModule, ROSSensorDataModule


class StateExecutor:
    """
    This class is for executing a single top level state or state machine.
    """

    def __init__(self, state, rate):
        self.state = state
        self.rate = rate if rate is not None else rospy.rate(5)

        self.controls = ROSControlsModule()
        self.sub_state = ROSStateEstimationModule()
        self.sensors = ROSSensorDataModule()

        self.exit_code = None

    @staticmethod
    def t():
        return rospy.Time.now().to_sec()

    def run(self):
        """
        This function will execute the state or state machine, and block until it terminates.
        """
        self.state.initialize(self.t(), self.controls, self.sub_state, None, self.sensors)

        while not rospy.is_shutdown():
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

            self.state.process(self.t(), self.controls, self.sub_state, None, self.sensors)

            if self.state.has_completed():
                self.state.finalize(self.t(), self.controls, self.sub_state, None, self.sensors)
                self.exit_code = self.state.exit_code()
                return

        # This will only occur if ROS is shutdown externally, such as Ctrl+c from the command line
        # This is the only scenario where -1 will be returned as an exit_code
        self.controls.halt_and_catch_fire()
        self.exit_code = -1

    def get_exit_code(self):
        """
        :return: The exit code that the state/state machine terminated with.
        """
        return self.exit_code
