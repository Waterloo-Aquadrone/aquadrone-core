from path_planning.states.base_state import BaseState


class ExitCodeState(BaseState):
    """
    This state does not interact with the depth control or orientation systems.
    """

    def __init__(self, code):
        self.code = code

    def __repr__(self):
        return f'ExitCode({self.code})'

    def has_completed(self):
        return True

    def exit_code(self):
        return self.code
