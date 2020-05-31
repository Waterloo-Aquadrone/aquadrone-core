#!/usr/bin/env python

from path_planning.states.base_state import BaseState


class MarkovChainStateMachine(BaseState):
    """
    The exit code of this state machine is just the exit code of its last state. If you want different exit codes in
    different scenarios, use multiple ExitCodeState objects with different codes and map to them as the terminal states.
    """

    def __init__(self, name, states, state_mapping_dictionaries, starting_index=0):
        """
        Note it might be easier to construct the states and state_mapping_dictionaries as follows:
        states, mappings = zip((state1, mapping1),  # 0
                               (state2, mapping2),  # 1
                               (state3, mapping3))  # 2

        :param states:
        :param state_mapping_dictionaries: A list of dictionaries mapping exit codes of the corresponding state to the
                                           index of the next state.
                                           If the next state is -1, then the state machine will terminate.
        :param starting_index:
        """
        self.name = name
        self.states = states
        self.state_mapping_dictionaries = state_mapping_dictionaries
        self.idx = starting_index
        self.completed = False

    def state_name(self):
        return self.name + '/' + self.states[self.idx].state_name()

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.states[self.idx].initialize(t, controls, sub_state, world_state, sensors)
        print(self.state_name(), 'starting to execute a markov chain with', len(self.states), 'states')

    def process(self, t, controls, sub_state, world_state, sensors):
        state = self.states[self.idx]
        state.process(t, controls, sub_state, world_state, sensors)

        if state.has_completed():
            state.finalize(t, controls, sub_state, world_state, sensors)

            # Do not modify self.idx if it will result in -1!
            new_idx = self.state_mapping_dictionaries[self.idx][state.exit_code()]
            if new_idx == -1:
                self.completed = True
                print(self.name, 'completed via', state.state_name(), 'sub-state!')
                return

            self.idx = new_idx
            new_state = self.states[self.idx]
            print(self.name, 'switching from', state.state_name(), 'to', new_state.state_name())
            new_state.initialize(t, controls, sub_state, world_state, sensors)
            new_state.process(t, controls, sub_state, world_state, sensors)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def has_completed(self):
        return self.completed

    def exit_code(self):
        # return the exit code of the last state
        return self.states[self.idx].exit_code()
