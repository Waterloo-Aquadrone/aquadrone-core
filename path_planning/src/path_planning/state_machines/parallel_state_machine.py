from path_planning.states.base_state import BaseState
from path_planning.state_tree import Tree


class ParallelStateMachine(BaseState):
    """
    This state machine will simultaneously run all of the states assigned to it.
    Note that the states must take care not to interfere with each other.
    Ideally, there should be at most one state that interacts with each of
    the depth control, orientation stability, and movement command systems.

    Daemon states (similar to a daemon thread in Java) can optionally be provided.
    Daemon states will not prevent this state machine from terminating.
    If all non-daemon states have completed, then the state machine will be terminated,
    and any still running daemon states will be finalized. Thus can be useful for logging, for example.

    The exit code of this state machine is the exit code of the state machine that is
    last in the list of non-daemons states provided.
    """

    def __init__(self, name, states, daemon_states=None):
        self.name = name
        self.states = states
        self.daemon_states = daemon_states if daemon_states is not None else []
        self.completed = False
        self.finalized_states = [False] * (len(self.states) + len(self.daemon_states))

    def state_name(self):
        return self.name + \
               '/[' + \
               ', '.join(state.state_name() for state in self.states if not state.has_completed()) + \
               ', '.join(state.state_name() + ' (daemon)' for state in self.daemon_states if not state.has_completed()) + \
               ']'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.completed = False
        self.finalized_states = [False] * (len(self.states) + len(self.daemon_states))

        print(self.state_name(), 'starting to execute', len(self.states), 'states and',
              len(self.daemon_states), 'daemon states in parallel')
        for state in self.states + self.daemon_states:
            state.initialize(t, controls, sub_state, world_state, sensors)

    def process(self, t, controls, sub_state, world_state, sensors):
        for idx, (state, finalized) in enumerate(zip(self.states + self.daemon_states, self.finalized_states)):
            if finalized:
                continue
            if state.has_completed():
                state.finalize(t, controls, sub_state, world_state, sensors)
                self.finalized_states[idx] = True
            else:
                state.process(t, controls, sub_state, world_state, sensors)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        for state, finalized in zip(self.states + self.daemon_states, self.finalized_states):
            if not finalized:
                state.finalize(t, controls, sub_state, world_state, sensors)

    def has_completed(self):
        # do not consider daemon states when testing for completion
        for state in self.states:
            if not state.has_completed():
                return False
        print(self.state_name(), 'completed!')
        return True

    def exit_code(self):
        # return the exit code of the last non-daemon state
        return self.states[-1].exit_code()

    def get_tree(self, depth=0):
        return Tree(name=self.name,
                    children=[child.get_tree(depth=depth+1) for child in self.states],
                    daemon=[child.get_tree(depth=depth+1) for child in self.daemon_states],
                    nodeType="ParallelState",
                    depth=depth)
