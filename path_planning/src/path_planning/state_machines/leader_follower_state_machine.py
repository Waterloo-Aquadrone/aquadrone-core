#!/usr/bin/env python

from path_planning.states.base_state import BaseState


class LeaderFollowerStateMachine(BaseState):
    """
    This state machine will simultaneous run the leader state and the follower state.
    Once the leader state exits, the follower state will also be forced to exit (by calling its finalize method).
    Note that the states must take care not to interfere with each other.
    Ideally, there should be at most one state that interacts with each of
    the depth control, orientation stability, and movement command systems.

    The exit code of this state machine is the exit code of the leader state.

    This is useful for attaching a state for logging or debugging data to a main leader state.
    """

    def __init__(self, name, leader_state, follower_state):
        self.name = name
        self.leader = leader_state
        self.follower = follower_state

    def state_name(self):
        return self.name + '/[' + self.leader.state_name() + ', ' + 'follower:' + self.follower.state_name() + ']'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        print(self.state_name(), 'starting to execute leader and follower states in parallel')
        for state in [self.leader, self.follower]:
            state.initialize(t, controls, sub_state, world_state, sensors)

    def process(self, t, controls, sub_state, world_state, sensors):
        if self.leader.has_completed():
            self.finalize(t, controls, sub_state, world_state, sensors)
        else:
            self.leader.process(t, controls, sub_state, world_state, sensors)
            self.follower.process(t, controls, sub_state, world_state, sensors)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.states[0].finalize(t, controls, sub_state, world_state, sensors)
        self.states[1].finalize(t, controls, sub_state, world_state, sensors)

    def has_completed(self):
        return self.leader.has_completed()

    def exit_code(self):
        # return the exit code of the leader state
        return self.leader.exit_code()
