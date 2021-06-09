import rospy
from .ros_modules import ROSControlsModule, ROSStateEstimationModule, \
    ROSWorldEstimationModule, ROSSensorDataModule
from aquadrone_math_utils.ros_utils import ros_time


class StateExecutor:
    """
    This class is for executing a single top level state or state machine.
    """

    def __init__(self, state, rate=None):
        self.state = state
        self.rate = rate if rate is not None else rospy.Rate(5)

        self.controls = ROSControlsModule()
        self.sub_state = ROSStateEstimationModule()
        self.world_state = ROSWorldEstimationModule()
        self.sensors = ROSSensorDataModule()

        self.has_completed = False
        self._exit_code = None
        rospy.on_shutdown(self.shutdown_hook)

    def run(self):
        """
        This function will execute the state and block until it terminates.
        """
        print(f'StateExecutor starting to execute top level state: {repr(self.state)}')
        self.state.initialize(ros_time(), self.controls, self.sub_state, self.world_state, self.sensors)

        while not rospy.is_shutdown():
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

            self.state.process(ros_time(), self.controls, self.sub_state, self.world_state, self.sensors)

            if self.state.has_completed():
                self.state.finalize(ros_time(), self.controls, self.sub_state, self.world_state, self.sensors)
                self._exit_code = self.state.exit_code()

                # still need to halt controls to send the thruster shutdown request
                self.controls.halt_and_catch_fire()
                self.has_completed = True
                return

        # This will only occur if ROS is shutdown externally, such as Ctrl+c from the command line
        # This is the only scenario where -1 will be returned as an exit_code
        self._exit_code = -1
        self.controls.halt_and_catch_fire()
        self.has_completed = True

    def exit_code(self):
        """
        :return: The exit code that the state terminated with.
        """
        return self._exit_code

    def shutdown_hook(self):
        if not self.has_completed:
            # This can occur, for example, if the underlying state crashes
            self.controls.halt_and_catch_fire()
            self.has_completed = True
