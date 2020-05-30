import cv2
import numpy as np
from simple_pid import PID

from path_planning.states.base_state import BaseState


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

    def process(self, t, controls, sub_state, world_state, sensors):
        dt = t - self.last_t
        self.last_t = t

        yaw = sub_state.get_submarine_state().orientation_rpy.z
        controls.set_yaw_goal(yaw + dt * 2*np.pi * 0.05)
        controls.planar_move_command(Fy=0, Fx=0.05)

        image = sensors.get_main_cam_image()
        mask = cv2.inRange(image, self.color_low, self.color_high)

        num = np.count_nonzero(mask)

        self.num_matching_pixels = num

        print("Num: %d" % num)

        cv2.imshow('image', image)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)

    def finalize(self, t, controls, sub_state, world_state, sensors):
        self.num_matching_pixels = 0
        controls.planar_move_command(Fy=0)

    def has_completed(self):
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

    def process(self, t, controls, sub_state, world_state, sensors):
        dt = t - self.last_t
        self.last_t = t

        #yaw = sub_state.get_submarine_state().orientation_rpy.z
        #controls.set_yaw_goal(yaw - dt * 2*np.pi * 0.05)

        image = sensors.get_main_cam_image()
        mask = cv2.inRange(image, self.color_low, self.color_high)

        indices = np.where(mask!= [0])
        avg_x = np.mean(indices[1])

        dx = (avg_x - 320) / 320.0

        yaw = sub_state.get_submarine_state().orientation_rpy.z
        d_yaw = -dt * 2*np.pi * 0.1 * dx
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

    def finalize(self, t, controls, sub_state, world_state, sensors):
        controls.planar_move_command(Fx=0)

    def completed(self):
        return self.has_lost_pole() or self.at_pole()

    def exit_code(self):
        """

        :return: 0 if the pole was successfully reached, 1 if the pole was lost.
        """
        if self.at_pole():
            return 0
        else:
            return 1
        
    def has_lost_pole(self):
        return self.num_matching_pixels < 4000

    def at_pole(self):
        return self.time_close > 3
