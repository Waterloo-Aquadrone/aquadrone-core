from path_planning.states.base_state import BaseState


class WaitingState(BaseState):
    """
    This state does not interact with the depth control or orientation systems.
    """

    def __init__(self, delay):
        self.delay = delay
        self.start_time = None
        self.completed = False

    @staticmethod
    def state_name():
        return "waiting_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.start_time = t

    def process(self, t, controls, sub_state, world_state, sensors):
        if t - self.start_time > self.delay:
            self.completed = True
            print(self.state_name(), 'completed waiting!')

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        return self.completed
