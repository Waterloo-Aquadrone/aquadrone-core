import math
from data_structures import Gate
from pole_finder_demo_states import BaseState
from ros_modules import ROSControlsModule
class GoThroughGate(BaseState):
    def __init__(self,do_roll,go_left,go_right):
        self.do_roll = do_roll
        self.go_left = go_left
        self.go_right = go_right

    def state_name(self):
        return "init_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        """
        sub_state
        """
        sub_pos = sub_state.get_submarine_state().position
        gate_pos = world_state.get_gate_state().position
        controls.set_depth_goal(gate_pos.z)
        ##for yaw
        x_coord = gate_pos.x - sub_pos.x
        y_coord = gate_pos.y - sub_pos.y
        x = (y_coord)/(x_coord)
        angle =  math.atan(x)
        controls.set_yaw_goal(angle)
        ##if(self.do_roll ):
        while(sub_pos.x <= gate_pos.x):
            planar_move_command(1,0,0)
        ##when the sub reaches the gate, it pitches
        planar_move_command(0,1,0)
        planar_move_command(0,-2,0)
        ##pitch done 
        planar_move_command(0,1,0)
        if(self.go_right):
            planar_move_command(1,0,0)
        elif(self.go_left):
            planar_move_command(1,0,0)


