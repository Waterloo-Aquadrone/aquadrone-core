from path_planning.states.base_state import BaseState


class TimedStateMachine(BaseState):
    """
    The exit code of this state machine is 10 if it times out.
    Otherwise, it is the exit code of the state that it is wrapping/
    """

    def __init__(self, state, timeout, timeout_exit_code=10):
        self.state = state
        self.timeout = timeout
        self.start_time = 0
        self.timed_out = False
        self.timeout_exit_code = timeout_exit_code

    def __repr__(self):
        return f'TimedStateMachine({repr(self.state)}, timeout={self.timeout}, ' \
               f'timeout_exit_code={self.timeout_exit_code})'

    def __str__(self):
        return f'TimedStateMachine(timeout={self.timeout}, timeout_exit_code={self.timeout_exit_code})'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.start_time = t
        self.timed_out = False

        print(self, 'starting to execute with timeout of', self.timeout, 'seconds')
        self.state.initialize(t, controls, sub_state, world_state, sensors)

    def process(self, t, controls, sub_state, world_state, sensors):
        if t - self.start_time > self.timeout:
            self.timed_out = True
            print(self.state, 'timed out after', self.timeout, 'seconds!')
        else:
            self.state.process(t, controls, sub_state, world_state, sensors)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.state.finalize(t, controls, sub_state, world_state, sensors)

    def has_completed(self):
        return self.timed_out or self.state.has_completed()

    def exit_code(self):
        return self.timeout_exit_code if self.timed_out else self.state.exit_code()
