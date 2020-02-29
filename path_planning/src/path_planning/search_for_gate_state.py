from pole_finder_demo_states import BaseState

class SearchForGate:
    def __init__(self, max_allowable_depth=5):
        self.Tz = 2
        self.forward = 1
        self.zedd_view = 4
        self.max_allowable_depth = max_allowable_depth
        self.gate_is_found = False
        self.current_state = 0
        self.starting_yaw = 0

    def state_name(self):
        return "search_for_gate_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        controls.set_depth_goal(self.max_allowable_depth/2)
        self.starting_yaw = sub_state.get_submarine_state.orientation_rpy.z
        controls.planar_move_command(Tz=self.Tz)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        controls.planar_move_command(Fx=0,Fy=0,Tz=0)
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        if(world_state.get_gate_state is not None):
            self.gate_is_found = True
        else if(self.current_state == 0):
            if(sub_state.get_submarine_state.orientation_rpy.z < self.starting_yaw):
                self.current_state = 1
        else if(self.current_state == 1):
            if(sub_state.get_submarine_state.orientation_rpy.z > self.starting_yaw):
                self.current_state = 2
                self.last_pos = sub_state.get_submarine_state.position
                controls.planar_move_command(Fx=self.forward,Fy=0,Tz=0)
        else if(self.current_state == 2):
            if((sub_state.get_submarine_state.position - self.last_pos).xy_magnitude() > self.zedd_view):
                self.current_state = 0
                controls.planar_move_command(Tz=self.Tz)


    def gate_is_found(self):
        return self.gate_is_found
