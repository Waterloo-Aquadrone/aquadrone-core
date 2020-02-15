import math
from data_structures import Gate
from pole_finder_demo_states import BaseState
from ros_modules import ROSControlsModule
class GoThroughGate(BaseState):
    def __init__(self,do_roll,go_left,go_right):
        self.do_roll = do_roll
        self.go_left = go_left
        self.go_right = go_right
        self.gotten_to_gate = False
        self.got_through_gate = False

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
        x_c = gate_pos.x - sub_pos.x
        y_c = gate_pos.y - sub_pos.y
        distance_between_gate = math.sqrt((x_c*x_c)+(y_c*y_c))
                ##for yaw
        if(distance_between_gate > 0.1 and self.gotten_to_gate == False):
            x = (x_c)/(y_c)
            angle =  math.atan(x)
            planar_move_command(0,0,0)
            controls.set_yaw_goal(angle)
            if(abs(sub_state.get_submarine_state().orientation_rpy.z - angle) < 0.1):
                planar_move_command(1,0,0)
            return
        else:
            self.gotten_to_gate = True
        ##when the sub reaches the gate, it pitches
        # planar_move_command(0,1,0)
        # planar_move_command(0,-2,0)
        #pitch done 
        # planar_move_command(0,1,0)
                
        ##straighten the submarine
                
        
        # if(self.go_right):
        #     planar_move_command(1,0,0)
        # elif(self.go_left):
        #     planar_move_command(1,0,0)
                
        length = sublength
        if(distance_between_gate < length and self.gotten_to_gate == True):
            planar_move_command(1,0,0)
        else:
            self.got_through_gate = True
        return
        
    def gone_through_gate():
        return self.got_through_gate



