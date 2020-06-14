#!/usr/bin/env python

from path_planning.states.base_state import BaseState


class ParallelStateMachine(BaseState):
    """
    This state machine will simultaneous run all of the states assigned to it.
    Note that the states must take care not to interfere with each other.
    Ideally, there should be at most one state that interacts with each of
    the depth control, orientation stability, and movement command systems.

    The exit code of this state machine is the exit code of the state machine that is last in the list provided.
    """

    def __init__(self, name, states):
        self.name = name
        self.states = states
        self.idx = 0
        self.completed = False
        self.finalized_states = [False] * len(states)

    def state_name(self):
        return self.name + \
               '/[' + ', '.join(state.state_name() for state in self.states if not state.has_completed()) + \
               ']'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.idx = 0
        self.completed = False
        self.finalized_states = [False] * len(self.states)

        print(self.state_name(), 'starting to execute', len(self.states), 'states in parallel')
        for state in self.states:
            state.initialize(t, controls, sub_state, world_state, sensors)

    def process(self, t, controls, sub_state, world_state, sensors):
        for idx, (state, finalized) in enumerate(zip(self.states, self.finalized_states)):
            if finalized:
                continue
            if state.has_completed():
                state.finalize(t, controls, sub_state, world_state, sensors)
                self.finalized_states[idx] = True
            else:
                state.process(t, controls, sub_state, world_state, sensors)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        for state, finalized in zip(self.states, self.finalized_states):
            if not finalized:
                state.finalize(t, controls, sub_state, world_state, sensors)

    def has_completed(self):
        for state in self.states:
            if not state.has_completed():
                return False
        print(self.state_name(), 'completed!')
        return True

    def exit_code(self):
        # return the exit code of the last state
        return self.states[-1].exit_code()
