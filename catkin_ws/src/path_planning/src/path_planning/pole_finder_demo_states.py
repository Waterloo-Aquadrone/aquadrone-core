import math
import cv2
import numpy as np
import scipy.misc
from simple_pid import PID

class BaseState(object):
    def __init__(self):
        pass

    def state_name(self):
        return "base_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        # Set up anything that needs initializing
        # Run EACH time the state is chosen as the next state
        # process(...) will be called with the next available data
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        # Clean up anything necessary
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        # Regular tick at some rate
        pass

    # Expose functions to identify when a state should exit
    # Ex: has_timed_out, has_lost_track_of_[some object]


class InitialState(object):
    def __init__(self):
        pass

    def state_name(self):
        return "init_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        pass


class GoToDepthState(BaseState):
    def __init__(self, d):
        self.depth_goal = d
        self.current_depth = 0

        self.time_at_depth = 0
        self.last_t = 0

    def state_name(self):
        return "go_to_depth_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        controls.set_depth_goal(self.depth_goal)
        self.last_t = t

    def finalize(self, t, controls, sub_state, world_state, sensors):
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        self.current_depth = -sub_state.get_submarinne_state().position.z
        dt = t - self.last_t
        self.last_t = t

        depth_err = abs(self.current_depth - self.depth_goal)

        if depth_err < 0.25:
            self.time_at_depth = self.time_at_depth + dt
        else:
            self.time_at_depth = 0

        print("Depth err (abs): %f" % depth_err)

    def depth_is_reached(self):
        return self.time_at_depth > 5


class ColoredPoleFinderState(BaseState):
    def __init__(self, color_low, color_high):
        self.color_low = np.asarray(color_low, dtype=np.uint8)
        self.color_high = np.asarray(color_high, dtype=np.uint8)

        self.image = None
        self.last_t = 0

        self.num_matching_pixels = 0

    def state_name(self):
        return "colored_pole_finder_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.last_t = t

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.num_matching_pixels = 0
        controls.planar_move_command(Fy=0)

    def process(self, t, controls, sub_state, world_state, sensors):
        dt = t - self.last_t
        self.last_t = t

        yaw = sub_state.get_submarinne_state().orientation_rpy.z
        controls.set_yaw_goal(yaw + dt * 2*math.pi * 0.05)
        controls.planar_move_command(Fy=-0, Fx=0.05)

        image = sensors.get_main_cam_image()
        mask = cv2.inRange(image, self.color_low, self.color_high)

        num = np.count_nonzero(mask)

        self.num_matching_pixels = num

        print("Num: %d" % num)

        
        cv2.imshow('image', image)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)
        


    def have_found_pole(self):
        return self.num_matching_pixels > 5000


class ColoredPoleApproacherState(BaseState):
    def __init__(self, color_low, color_high):
        self.color_low = np.asarray(color_low, dtype=np.uint8)
        self.color_high = np.asarray(color_high, dtype=np.uint8)

        self.image = None
        self.last_t = 0

        self.num_matching_pixels = 25000

        self.time_close = 0

        self.fx_pid = PID(1, 0, 10)
        self.fx_pid.output_limits = (-0.5, 0.5)
        self.fx_pid.setpoint = 0

    def state_name(self):
        return "colored_pole_approacher_state"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.last_t = t
        self.time_close = 0

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.num_matching_pixels = 0
        controls.planar_move_command(Fx=0)

    def process(self, t, controls, sub_state, world_state, sensors):
        dt = t - self.last_t
        self.last_t = t

        #yaw = sub_state.get_submarinne_state().orientation_rpy.z
        #controls.set_yaw_goal(yaw - dt * 2*math.pi * 0.05)

        image = sensors.get_main_cam_image()
        mask = cv2.inRange(image, self.color_low, self.color_high)

        indices = np.where(mask!= [0])
        avg_x = np.mean(indices[1])

        dx = (avg_x - 320) / 320.0

        yaw = sub_state.get_submarinne_state().orientation_rpy.z
        d_yaw = -dt * 2*math.pi * 0.1 * dx
        print("d_yaw: %f" % d_yaw)

        if not np.isnan(d_yaw):
            controls.set_yaw_goal(yaw + d_yaw)

        num = np.count_nonzero(mask)

        self.num_matching_pixels = num

        rel_num = (num - 25000.0) / 10000.0
        Fx = self.fx_pid(rel_num)
        controls.planar_move_command(Fx=Fx)
        print("Num: %d" % num)
        print("Fx: %f" % Fx)

        pix_err = abs(num - 25000)
        if pix_err < 1000:
            self.time_close = self.time_close + dt
        else:
            self.time_close = 0

        
        cv2.imshow('image', image)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)
        
    def has_lost_pole(self):
        return self.num_matching_pixels < 4000

    def at_pole(self):
        return self.time_close > 3
