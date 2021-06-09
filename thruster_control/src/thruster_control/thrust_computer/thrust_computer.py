import rospy
import rospkg
from aquadrone_msgs.msg import MotorControls
from thruster_control.thrust_computer.movement_command_collector import MovementCommandCollector
import numpy as np
from scipy.optimize import linprog, minimize


class ThrustComputer:
    POUNDS_TO_NEWTONS = 4.44822

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

        # TODO: generalize this in a way so that it works better with simulated and real thrusters.
        rospack = rospkg.RosPack()
        config_path = rospack.get_path('thruster_control') + "/config/"
        data = np.genfromtxt(config_path + "pwm_thrust_conversion.csv", delimiter=",", names=True)
        self.motor_thrusts_newtons = data['thrust_lbs'] * ThrustComputer.POUNDS_TO_NEWTONS
        self.motor_efficiency = data['efficiency_out_of_100'] / 100
        self.k = 5
        self.max_reverse_thrust = 4.1 * self.POUNDS_TO_NEWTONS  # Newtons
        self.max_forward_thrust = 5.2 * self.POUNDS_TO_NEWTONS  # Newtons

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

    def control_loop(self):
        wrench_list = self.mcc.get_recent_wrenches()
        final_thrusts = self.optimize_thrusts_2(wrench_list)

        # Publish commands for new thrust
        self.publish_command(final_thrusts)

    def optimize_thrusts(self, wrench_list):
        thrusts_list = [self.config.wrench_to_thrusts(wrench) for wrench in wrench_list]

        # for now use sequential powers of 10 for the weights
        objective_weights = [-10 ** i for i in range(len(thrusts_list) - 1, -1, -1)]

        A_top = np.column_stack(thrusts_list)
        A_bottom = np.negative(A_top)
        A = np.vstack((A_top, A_bottom))

        b_top = np.ones(A_top.shape[0]) * self.max_forward_thrust
        b_bottom = np.ones(A_bottom.shape[0]) * self. max_reverse_thrust
        b = np.hstack((b_top, b_bottom))

        result = linprog(objective_weights, A_ub=A, b_ub=b, bounds=[(0, 1)] * len(thrusts_list))

        scaling_coefficients = result.x
        overall_thrusts = sum([coefficient * thrusts
                               for coefficient, thrusts in zip(scaling_coefficients, thrusts_list)])
        return overall_thrusts

    def calculate_motor_efficiency(self, motor_thrust):
        """
        Calculates the efficiency that the BlueRobotics Thruster would operate at when producing the specified thrust.

        :param motor_thrust: Can be an individual motor thrust, or an array of motor thrusts.
        """
        return np.interp(motor_thrust, self.motor_thrusts_newtons, self.motor_efficiency)

    def get_efficiency_error(self, T, wrench_list):
        W_total = self.config.thrusts_to_wrench(T)

        coefficients = []
        for W_i in wrench_list:
            c = np.clip(np.dot(W_total, W_i) / np.dot(W_i, W_i), 0, 1)
            if np.isnan(c):  # this will only occur if W_i is zero
                c = 1
            coefficients.append(c)
            W_total -= c * W_i

        energy_usage = sum((1 / 2) * T ** 2 * self.calculate_motor_efficiency(T))
        objective_weights = np.array([-10 ** i for i in range(len(wrench_list) - 1, -1, -1)])
        return np.dot(objective_weights, np.array(coefficients)) + self.k * energy_usage

    def optimize_thrusts_2(self, wrench_list):
        ideal_thrust = self.config.wrench_to_thrusts(sum(wrench_list))
        ideal_thrust = np.clip(ideal_thrust, -self.max_reverse_thrust, self.max_forward_thrust)
        result = minimize(self.get_efficiency_error, ideal_thrust,
                          bounds=len(ideal_thrust) * [(-self.max_reverse_thrust, self.max_forward_thrust)],
                          args=(wrench_list,))

        if not result.success:
            return self.optimize_thrusts(wrench_list)

        thrusts = result.x
        return thrusts

    def publish_command(self, thrusts):
        msg = MotorControls()
        msg.motorThrusts = [float(thrust) for thrust in thrusts]
        self.publisher.publish(msg)
