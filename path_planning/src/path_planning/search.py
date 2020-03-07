from .pole_finder_demo_states import BaseState


class Search:
    def __init__(self, max_allowable_depth=5):
        self.Tz = 2
        self.forward = 1
        self.is_found = False
        self.starting_yaw = 0

    def state_name(self):
        return "search_for_gate_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        controls.set_depth_goal(self.max_allowable_depth/2)
        self.starting_yaw = sub_state.get_submarine_state.orientation_rpy.z
        controls.planar_move_command(Tz=self.Tz)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        controls.planar_move_command(Fx=0, Fy=0, Tz=0)

    def process(self, t, controls, sub_state, world_state, sensors):
        if(world_state.get_gate_state is not None):
            self.gate_is_found = True
            return

        # For now, spiral for some amount
        yaw = sub_state.get_submarinne_state().orientation_rpy.z
        controls.set_yaw_goal(yaw + dt * 2*math.pi * 0.05)
        controls.planar_move_command(Fy=0, Fx=0.05)

    def did_find():
        return is_found
