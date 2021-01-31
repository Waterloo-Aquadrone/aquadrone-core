import numpy as np
from path_planning.states.base_state import BaseState
from aquadrone_math_utils.angle_math import abs_angle_difference


class TravelState(BaseState):
    """
    Moves the submarine to the target position, assuming there are no obstacles in the way.
    """
    k_p = np.array([10, 10])  # K_p terms for pure movement in x and y directions respectively
    k_p_prod = np.product(k_p)
    # Note negative sign for k_d must be applied manually
    k_d = np.array([0.1, 0.1])  # K_d terms for pure movement in x and y directions respectively
    k_d_prod = np.product(k_d)

    fine_control_threshold = 0.5  # meters

    def __init__(self, target_x=0, target_y=0, target_z=0, target_yaw=0, verbose=True):
        """
        This class assumes that the target roll and pitch are 0.
        Units are meters and degrees.
        """
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = np.radians(target_yaw)
        self.verbose = verbose
        self.completed = False

    def update_target(self, target_x=None, target_y=None, target_z=None, target_yaw=None):
        """
        Updates the target to the new location. Any parameters left as None will be ignored.
        """
        if target_x is not None:
            self.target_x = target_x
        if target_y is not None:
            self.target_y = target_y
        if target_z is not None:
            self.target_z = target_z
        if target_yaw is not None:
            self.target_yaw = target_yaw

    def state_name(self):
        return "travel"

    def initialize(self, t, controls, sub_state, world_state, sensors):
        self.completed = False
        if self.verbose:
            print('Starting to travel to: (x=' + str(self.target_x) + ', y=' + str(self.target_y) + ', z=' +
                  str(self.target_z) + ', yaw=' + str(self.target_yaw) + ')')

    def process(self, t, controls, sub_state, world_state, sensors):
        controls.set_movement_target(self.target_x, self.target_y)
        controls.set_depth_goal(self.target_z)

        sub = sub_state.get_submarine_state()
        displacement = np.array([self.target_x - sub.position.x,
                                 self.target_y - sub.position.y])
        dist = np.linalg.norm(displacement)
        v = np.array([sub.velocity.x, sub.velocity.y])

        # check to see if we are completed
        if dist < 0.1 and np.linalg.norm(v) < 0.1 and \
                abs_angle_difference(sub.orientation_rpy.z, self.target_yaw) < 10 and \
                np.degrees(np.abs(sub.angular_velocity.z)) < 5 and \
                abs(sub.position.z - self.target_z) < 0.1 and np.abs(sub.velocity.z) < 0.1:
            self.completed = True
            return

        # controls.set_depth_goal(self.target_z)
        #
        # angle_to_target = np.arctan2(displacement[1], displacement[0])
        # # yaw to either face target or target angle, depending on how close we are
        # if dist > TravelState.fine_control_threshold:
        #     controls.set_yaw_goal(angle_to_target)
        #     # if we are not yet facing the right direction, focus on that and leave x-y motion until later
        #     if abs_angle_difference(sub.orientation_rpy.z, angle_to_target) > 5:
        #         controls.planar_move_command(0, 0)
        #         return
        # else:
        #     controls.set_yaw_goal(self.target_yaw)
        #
        # # Calculations for the applied force
        #
        # v_target = sum(v * displacement) * displacement / dist ** 2  # vector velocity in the direction of target
        # v_perp = v - v_target  # vector velocity perpendicular to target
        #
        # heading = sub.orientation_rpy.z - angle_to_target
        # heading_trig = np.array([np.sin(heading), np.cos(heading)])
        # # interpolate k_p and k_d values using elliptical fit between the pure x and pure y values based on heading
        # k_p = TravelState.k_p_prod / np.linalg.norm(TravelState.k_p * heading_trig)
        # k_d = TravelState.k_d_prod / np.linalg.norm(TravelState.k_d * heading_trig)
        # k_d_perp = TravelState.k_d_prod / np.linalg.norm(TravelState.k_d[::-1] * heading_trig)
        #
        # # TODO: v_perp is undamped in this scheme and will (in theory) oscillate sinusoidally indefinitely
        # absolute_forces = k_p * displacement - k_d * v_target - k_d_perp * v_perp
        #
        # sin = np.sin(sub.orientation_rpy.z)
        # cos = np.cos(sub.orientation_rpy.z)
        # abs_to_sub_transform = np.array([[cos, -sin],
        #                                  [sin, cos]])
        # relative_forces = np.dot(abs_to_sub_transform, absolute_forces)
        #
        # controls.planar_move_command(relative_forces[0], relative_forces[1])

    def has_completed(self):
        return self.completed
