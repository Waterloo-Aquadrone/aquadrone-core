import numpy as np
from path_planning.states.base_state import BaseState


class ThrusterTestState(BaseState):
    """
    This state does not interact with the depth control or orientation systems.
    This state directly sends commands to the /motor_command topic, bypassing the thrust_computer node (which must not be running for this to work).
    """

    def __init__(self, thruster_count=8, amplitude=3, period=5):
        """
        Tests each of the thrusters in sequence with a sinusoidal thrust output with the given amplitude and period.

        :param thruster_count: The number of thrusters to test.
        :param amplitude: The amplitude of the thrust in newtons.
        :param period: The amount of time that each thruster will be tested for.
        """
        self.thruster_count = thruster_count
        self.thrust_amplitude = amplitude
        self.period = period
        self.k = 2 * np.pi / period
        self.start_time = 0
        self.completed = False

    def __repr__(self):
        return f'ThrusterTestState(thruster_count={self.thruster_count}, amplitude={self.thrust_amplitude}, ' \
               f'period={self.period})'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.start_time = t
        self.completed = False
        print(self, 'starting to test', self.thruster_count, 'thrusters')

    def process(self, t, controls, sub_state, world_state, sensors):
        thruster_index = int((t - self.start_time) / self.period)  # round down
        if thruster_index >= self.thruster_count:
            self.completed = True
            return
         
        thrusts = np.zeros(self.thruster_count)
        thrusts[thruster_index] = self.thrust_amplitude * np.sin(self.k * (t - self.start_time))
        controls.send_direct_motor_thrusts(thrusts)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        controls.send_direct_motor_thrusts(np.zeros(self.thruster_count))

    def has_completed(self):
        return self.completed
