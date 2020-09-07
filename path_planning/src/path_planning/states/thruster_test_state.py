import numpy as np
from path_planning.states.base_state import BaseState


class ThrusterTestState(BaseState):
    """
    This state does not interact with the depth control or orientation systems.
    This state directly sends commands to the /motor_command topic, bypassing the thrust_computer node (which must not be running for this to work).
    """

    def __init__(self, thruster_count=8, thrust_amplitude=3, thrust_period=5):
        self.thruster_count = thruster_count
        self.thrust_amplitude = thrust_amplitude
        self.k = 2 * np.pi / thrust_period
        self.start_time = 0
        self.completed = False

    @staticmethod
    def state_name():
        return "thruster_test_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.start_time = t
        self.completed = False
        print(self.state_name(), 'starting to test', self.thruster_count, 'thrusters')

    def process(self, t, controls, sub_state, world_state, sensors):
        thruster_index = int(t - self.start_time)  # round down
        if thruster_index == self.thruster_count:
            self.completed = True
            return
         
        thrusts = np.zeros(self.thruster_count)
        thrusts[thruster_index] = self.thruster_amplitude * np.sin(self.k * (t - self.start_time))
        controls.send_direct_motor_thrusts(thrusts)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        return self.completed
