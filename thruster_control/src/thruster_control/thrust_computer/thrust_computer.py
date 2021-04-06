# import rospy
# from aquadrone_msgs.msg import MotorControls
# from thruster_control.thrust_computer.movement_command_collector import MovementCommandCollector
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
        self.publisher = rospy.Publisher("motor_command", MotorControls, queue_size=0)

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

    def control_loop(self):
        wrench_list = self.mcc.get_recent_thrusts()
        thrusts_list = []

        for i in range(len(wrench_list)):
            thrusts_list.append(self.config.wrench_to_thrusts(wrench_list[i]))

        final_thrusts = self.optimize_thursts(self, thrusts_list)
        
        # A = thrusts_list[0]
        # B = thrusts_list[1]
        # C = thrusts_list[2]

        # thrusts = []
        # for i,j,k in zip(A, B, C):
        #     thrusts.append(i+j+k)

        # fwd_max = 5.2 * 4.44822
        # rev_max = -4.1 * 4.44822

        # requirements_met = True

        # for i in range(len(thrusts)):
        #     if thrusts[i] < rev_max or thrusts[i] > fwd_max:
        #         requirements_met = False
        #         break
        
        # if requirements_met == True:         
        #     # Publish commands thrust as everything is already in the range
        #     self.publish_command(thrusts)
        #     return

        # # Else continue to scale
        # max_limits = []
        # min_limits = []

        # max_scale = []
        # min_scale = []

        # for i in range(8):
        #     max_limits.append(fwd_max - A[i] - B[i])
        #     min_limits.append(rev_max - A[i] - B[i])

        #     max_scale.append(max_limits[i]/C[i])
        #     min_scale.append(min_limits[i]/C[i])
        
        # scale_c = 1.0

        # # we need to only consider positive values in this method of scaling
        # # negative values are for the other limitor
        # # means either max_scale limits it or min_scale limits it
        # for i in range(len(8)):
        #     if max_scale[i] > 0 and max_scale[i] < scale_c:
        #         scale_c = max_scale[i]
        #     if min_scale[i] > 0 and min_scale[i] < scale_c:
        #         scale_c = min_scale[i]
        
        # new_C = []
        # final_thrusts = []

        # for i in range(len(C)):
        #     new_C.append(C[i] * scale_c)
        #     final_thursts.append(A[i] + B[i] + new_C[i])

        # Publish commands for new thrust
        self.publish_command(final_thursts)
    @staticmethod
    def optimize_thursts(thrusts):
        C = [-100, -10, -1]
        
        t_one = np.array(thrusts[0])
        t_two = np.array(thrusts[1])
        t_three = np.array(thrusts[2])

        A_top = np.column_stack((t_one, t_two, t_three))
        A_bottom = np.negative(A_top)

        A = np.vstack((A_top,A_bottom))

        fwd_max = 5.2 * 4.44822
        rev_max = 4.1 * 4.44822

        b_fwd=np.ones(8)*fwd_max
        b_rev=np.ones(8)*rev_max

        b = np.hstack((b_fwd, b_rev))

        x_bounds = (0, 1)

        res = linprog(C, A_ub=A, b_ub=b, bounds=[x_bounds, x_bounds, x_bounds])

        scale_list = res.x

        thrust_one = t_one * scale_list[0]
        thrust_two = t_two * scale_list[1]
        thrust_three = t_three * scale_list[2]

        return [thrust_one, thrust_two, thrust_three]

    def publish_command(self, thrusts):
        msg = MotorControls()
        msg.motorThrusts = [float(thrust) for thrust in thrusts]
        self.publisher.publish(msg)
