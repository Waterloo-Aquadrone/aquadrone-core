#!/usr/bin/env python

from path_planning.states.base_state import BaseState


class BranchingStateMachine(BaseState):
    """
    This state machine runs the decision_state.
    Once the decision_state exits, it runs either the success_state or the failure_state depending on whether or not
    the decision_state's exit code matches the success_code.
    The exit code of this state machine is either the exit code of the success_state or of the failure_state
    depending on which one was run.
    """

    def __init__(self, name, decision_state, success_state, failure_state, success_code=0):
        self.name = name
        self.states = [decision_state, success_state, failure_state]
        self.success_code = success_code
        self.idx = 0
        self.completed = False

    def state_name(self):
        return self.name + '/' + self.states[self.idx].state_name()

    def initialize(self, t, controls, sub_state, world_state, sensors):
        print(self.state_name(), 'starting to execute')
        self.states[self.idx].initialize(t, controls, sub_state, world_state, sensors)

    def process(self, t, controls, sub_state, world_state, sensors):
        state = self.states[self.idx]
        state.process(t, controls, sub_state, world_state, sensors)

        if state.has_completed():
            if self.idx == 0:
                # Only manually finalize the decision state
                # The success/failure state will be finalized when this state machine is finalized
                state.finalize(t, controls, sub_state, world_state, sensors)

                self.idx = 1 if state.exit_code() == self.success_code else 2
                new_state = self.states[self.idx]
                print(self.name, 'switching from', state.state_name(), 'to', new_state.state_name())
                new_state.initialize(t, controls, sub_state, world_state, sensors)
                new_state.process(t, controls, sub_state, world_state, sensors)
            else:
                self.completed = True
                print(self.name, 'completed!')
                return

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.states[self.idx].finalize(t, controls, sub_state, world_state, sensors)

    def has_completed(self):
        return self.completed

    def exit_code(self):
        # return the exit code of the state that was run
        return self.states[self.idx].exit_code()
