#!/usr/bin/env python

from path_planning.states.base_state import BaseState


class ExitCodeState(BaseState):
    """
    This state does not interact with the depth control or orientation systems.
    """

    def __init__(self, code):
        self.code = code

    def state_name(self):
        return 'exit_code'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        return True

    def exit_code(self):
        return self.code
