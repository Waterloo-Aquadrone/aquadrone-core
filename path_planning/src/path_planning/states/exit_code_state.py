from path_planning.states.base_state import BaseState


class ExitCodeState(BaseState):
    """
    This state does not interact with the depth control or orientation systems.
    """

    def __init__(self, code):
        self.code = code

    def __repr__(self):
        return self.state_name()

    def state_name(self):
        return 'exit_code'

    def has_completed(self):
        return True

    def exit_code(self):
        return self.code
