import rospy
from aquadrone_msgs.msg import MotorControls
from thruster_control.thrust_computer.movement_command_collector import MovementCommandCollector
import numpy as np
from scipy.optimize import linprog


class ThrustComputer:
    """
    Creates a MovementCommandCollector to listen to the relevant topics for thrust commands.
    Calculates the required individual motor thrusts to achieve the desired Wrench (using the ThrusterConfiguration
    supplied on construction), and publishes the resulting MotorControls message to the motor_command topic.
    This node may be bypassed (not turned on) if fine grained control of individual thrusters is desired, such as for a
    diagnostic test.
    """

    def __init__(self, config, rate=None):
        self.config = config
        self.mcc = MovementCommandCollector()
        self.rate = rate if rate is not None else rospy.Rate(10)

        # Will need motor commands published for state estimation
        self.publisher = rospy.Publisher("motor_command", MotorControls, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

    def control_loop(self):
        wrench_list = self.mcc.get_recent_wrenches()
        thrusts_list = [self.config.wrench_to_thrusts(wrench) for wrench in wrench_list]
        final_thrusts = self.optimize_thrusts(thrusts_list)

        # Publish commands for new thrust
        self.publish_command(final_thrusts)

    @staticmethod
    def optimize_thrusts(thrusts_list):
        # for now use sequential powers of 10 for the weights
        objective_weights = [-10 ** i for i in range(len(thrusts_list) - 1, -1, -1)]

        A_top = np.column_stack(thrusts_list)
        A_bottom = np.negative(A_top)
        A = np.vstack((A_top, A_bottom))

        max_forward_thrust = 5.2 * 4.44822  # Newtons
        max_reverse_thrust = 4.1 * 4.44822  # Newtons

        b_top = np.ones(A_top.shape[0]) * max_forward_thrust
        b_bottom = np.ones(A_bottom.shape[0]) * max_reverse_thrust
        b = np.hstack((b_top, b_bottom))

        result = linprog(objective_weights, A_ub=A, b_ub=b, bounds=[(0, 1)] * len(thrusts_list))

        scaling_coefficients = result.x
        overall_thrusts = sum([coefficient * thrusts
                               for coefficient, thrusts in zip(scaling_coefficients, thrusts_list)])
        return overall_thrusts

    def publish_command(self, thrusts):
        msg = MotorControls()
        msg.motorThrusts = [float(thrust) for thrust in thrusts]
        self.publisher.publish(msg)
