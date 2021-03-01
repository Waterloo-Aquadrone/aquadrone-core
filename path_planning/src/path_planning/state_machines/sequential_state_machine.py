from path_planning.states.base_state import BaseState
from path_planning.state_tree import Tree


class SequentialStateMachine(BaseState):
    """
    The exit code of this state machine is just the exit code of its last state.
    """

    def __init__(self, name, states):
        self.name = name
        self.states = states
        self.idx = 0
        self.completed = False

    def state_name(self):
        return self.name + '/' + self.states[self.idx].state_name()

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.idx = 0
        self.completed = False

        print(self.state_name(), 'starting to execute', len(self.states), 'states sequentially')
        self.states[self.idx].initialize(t, controls, sub_state, world_state, sensors)

    def process(self, t, controls, sub_state, world_state, sensors):
        state = self.states[self.idx]
        state.process(t, controls, sub_state, world_state, sensors)

        if state.has_completed():
            # Do not modify self.idx if it will result in -1!
            if self.idx == len(self.states) - 1:
                self.completed = True
                print(self.name, 'completed!')
                return

            # Only manually finalize the state if it is not the last one
            # If the state is the last one, it will be finalized when this state machine is finalized
            state.finalize(t, controls, sub_state, world_state, sensors)

            self.idx += 1
            new_state = self.states[self.idx]
            print(self.name, 'switching from', state.state_name(), 'to', new_state.state_name())
            new_state.initialize(t, controls, sub_state, world_state, sensors)
            new_state.process(t, controls, sub_state, world_state, sensors)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.states[self.idx].finalize(t, controls, sub_state, world_state, sensors)

    def has_completed(self):
        return self.completed

    def exit_code(self):
        # return the exit code of the last state
        return self.states[-1].exit_code()

    def get_tree(self, depth=0):
        return Tree(name=self.name,
                    children=[child.get_tree(depth=depth+1) for child in self.states],
                    nodeType="SequentialState",
                    depth=depth)
