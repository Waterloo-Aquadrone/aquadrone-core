import numpy as np
import sympy as sp
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt


class Quaternion:
    """
    Throughout the Aquadrone code base, orientations are specified in one of 2 formats:
    1. Intrinsic ZYX Euler angles (yaw, pitch, and roll respectively).
    This corresponds to the human intuition of yaw, then pitch about new y axis, then roll about new x axis.
    2. Unit quaternion

    Angular velocities are specified in one of 2 formats:
    1. roll, pitch and yaw rates. These are in rad/s along the submarine's relative ZYX axes.
    2. A vector [Wx, Wy, Wz] which points in the direction of the rotation axis in the static
       coordinate system and has a magnitude equal to the rotation rate in rad/s (following the right hand rule).
    """
    def __init__(self, q_0=0, q_1=0, q_2=0, q_3=1):
        self.q_0 = q_0
        self.q_1 = q_1
        self.q_2 = q_2
        self.q_3 = q_3

    @staticmethod
    def from_real_imag(q_0, q_vec):
        return Quaternion(q_0, q_vec[0], q_vec[1], q_vec[2])

    @staticmethod
    def from_array(arr):
        return Quaternion(arr[0], arr[1], arr[2], arr[3])

    @staticmethod
    def from_scipy(rotation):
        x, y, z, w = rotation.as_quat()
        return Quaternion.from_array([w, x, y, z])

    @staticmethod
    def from_euler(roll=0, pitch=0, yaw=0):
        return Quaternion.from_scipy(Rotation.from_euler('ZYX', [yaw, pitch, roll]))

    def real(self):
        return self.q_0

    def imag(self):
        return np.array([self.q_1, self.q_2, self.q_3])

    def conj(self):
        return Quaternion.from_real_imag(self.real(), -self.imag())

    def rotate(self, x_vec):
        return self.conj() * Quaternion.from_real_imag(0, x_vec) * self

    def unrotate(self, x_vec_prime):
        return self * Quaternion.from_real_imag(0, x_vec_prime) * self.conj()

    def E(self):
        return np.array([[-self.q_1,  self.q_0, -self.q_3,  self.q_2],
                         [-self.q_2,  self.q_3,  self.q_0, -self.q_1],
                         [-self.q_3, -self.q_2,  self.q_1,  self.q_0]])

    def G(self):
        return np.array([[-self.q_1,  self.q_0,  self.q_3, -self.q_2],
                         [-self.q_2, -self.q_3,  self.q_0,  self.q_1],
                         [-self.q_3,  self.q_2, -self.q_1,  self.q_0]])

    def as_scipy(self):
        return Rotation.from_quat(np.array([self.q_1, self.q_2, self.q_3, self.q_0]))

    def as_array(self):
        return np.array([self.q_0, self.q_1, self.q_2, self.q_3])

    def as_matrix(self):
        return self.as_scipy().as_matrix()

    def as_euler(self):
        """
        Returns the roll, pitch, and yaw that corresponds to this quaternion.
        """
        return self.as_scipy().as_euler('ZYX')[::-1]

    def as_euler_sympy(self):
        """
        Function for converting quaternions to Euler angles using sympy.
        This should only be used for getting derivative information for the EKF.

        For other calculations, use as_euler instead.
        """
        # Implementation based on page 16 of this paper:
        # https://drive.google.com/file/d/1u2uIzvj4GLaSEUvjplONdsSUh_swtdne/view?usp=sharing
        w, x, y, z = self.as_array()

        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1 - 2.0 * (x * x + y * y)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1 - 2.0 * (y * y + z * z)

        # pitch (y-axis rotation)
        sinp = 2.0 * (-x * z + w * y)

        # return in the order: roll, pitch, yaw
        if np.abs(sinp) == 1:
            return [
                0,
                sp.sign(sinp) * np.pi / 2,
                -2 * sp.sign(sinp) * sp.atan2(x, w).evalf()
            ]
        else:
            return [
                sp.atan2(sinr_cosp, cosr_cosp).evalf(),
                sp.asin(sinp).evalf(),
                sp.atan2(siny_cosp, cosy_cosp).evalf()
            ]

    @staticmethod
    def get_quat_to_euler_jacobian_func():
        q_vars = np.asarray(sp.symbols('q_:4', real=True))
        euler_angles = Quaternion.from_array(q_vars).as_euler_sympy()

        jacobian_matrix = [[sp.lambdify([q_vars], sp.diff(euler_angle, q_i)) for q_i in q_vars]
                           for euler_angle in euler_angles]

        return lambda quaternion_array: np.array([[func(quaternion_array) for func in jacobian_row]
                                                  for jacobian_row in jacobian_matrix])

    def quat_to_euler_jacobian(self):
        """
        Returns the jacobian matrix for the quaternion to euler transformation, evaluated for this quaternion.

        Note that this function is defined after the definition of the Quaternion class,
        using Quaternion.get_quat_to_euler_jacobian_func.
        """
        pass

    def normalize(self):
        arr = self.as_array()
        return Quaternion.from_array(arr / np.linalg.norm(arr))

    def __mul__(self, other):
        if type(other) is Quaternion:
            real = self.real() * other.real() - np.sum(self.imag() * other.imag())
            imag = self.real() * other.imag() + other.real() * self.imag() + np.cross(self.imag(), other.imag())
            return Quaternion.from_real_imag(real, imag)
        if type(other) is int or type(other) is float:
            return Quaternion(other * self.q_0, other * self.q_1, other * self.q_2, other * self.q_3)
        raise NotImplemented

    def __add__(self, other):
        if type(other) is Quaternion:
            return Quaternion(self.q_0 + other.q_0, self.q_1 + other.q_1,
                              self.q_2 + other.q_2, self.q_3 + other.q_3)
        raise NotImplemented

    def __str__(self):
        return f'[{self.q_0}, {self.q_1}, {self.q_2}, {self.q_3}]'

    def __repr__(self):
        return f'Quaternion({self.q_0}, {self.q_1}, {self.q_2}, {self.q_3})'

    @staticmethod
    def get_omega(q, q_dot):
        # noinspection PyCallingNonCallable
        return (2 * q_dot * q.conj()).imag()

    @staticmethod
    def get_omega_prime(q, q_dot):
        # noinspection PyCallingNonCallable
        return (2 * q.conj() * q_dot).imag()

    @staticmethod
    def get_q_dot_from_omega(q, omega_vec):
        return (1/2) * Quaternion.from_real_imag(0, omega_vec) * q

    @staticmethod
    def get_q_dot_from_omega_prime(q, omega_vec_prime):
        return (1 / 2) * q * Quaternion.from_real_imag(0, omega_vec_prime)


# Define the quat_to_euler_jacobian instance function. This is done outside the class definition so that we can use the
# Quaternion.get_quat_to_euler_jacobian_func function. This allows us to just compute the derivative once and improve
# performance with sp.lambdify
quaternion_to_euler_jacobian_func = Quaternion.get_quat_to_euler_jacobian_func()
Quaternion.quat_to_euler_jacobian = lambda self: quaternion_to_euler_jacobian_func(self.as_array())


def test_controller():
    """
    Attempt to implement a Quaternion-based PD controller.
    The system is numerically integrated to see if the controller successfully stabilizes to the target orientation.
    The notation used is based off of this paper:
    https://drive.google.com/file/d/12m8hsQJ-EKk8vDQYVwyyOT0U7RaDDAJw/view?usp=sharing
    """
    data = []

    target_quat = Quaternion.from_scipy(Rotation.from_euler('ZYX', np.array([0, 0, 0])))
    dt = 0.001
    # inertia matrix based on SolidWorks model
    J = np.array([[2850, 1, -25],
                  [1, 3800, 200],
                  [-25, 200, 2700]])
    J_inv = np.linalg.inv(J)

    # Create starting quaternion and angular velocity
    q = Quaternion.from_array(np.random.random(4)).normalize()
    omega_vec_prime = np.random.random(3)

    # PID coefficients
    k_p = np.array([75, 75, 40])
    k_d = np.array([150, 1000, 2000])

    for _ in range(int(400 / dt)):
        # calculate torque based on roll, pitch, yaw
        # tau_vec_prime = np.dot(q.as_matrix(), -k_p * (q.as_rpy() - target_quat.as_rpy()) - k_d * omega_vec_prime)

        # calculate torque directly from quaternions
        # based on this paper: https://drive.google.com/file/d/1E2Oj3E-XInCSGpyhnBP0luWHWV0s0z9m/view?usp=sharing
        quat_error = target_quat * q.conj()
        axis_error = quat_error.imag() * (1 if (quat_error.real() > 0) else -1)
        tau_vec_prime = -k_p * axis_error - k_d * omega_vec_prime

        q_dot = Quaternion.from_array((-1/2) * np.dot(q.G().transpose(), omega_vec_prime))
        omega_vec_prime_dot = np.dot(J_inv, tau_vec_prime) - \
            np.dot(J_inv, np.cross(omega_vec_prime, np.dot(J, omega_vec_prime)))
        q += q_dot * dt
        q = q.normalize()
        omega_vec_prime += omega_vec_prime_dot * dt

        data.append(q.as_array())
        if np.linalg.norm(q_dot.as_array()) > 100_000:
            print('Warning: unstable!')
            break

    for i in range(4):
        plt.plot(dt * np.arange(len(data)), [elem[i] for elem in data], label=f'$q_{i}$')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.show()

    print('Target Quaternion:', target_quat)
    print('Final Quaternion Achieved:', q)


if __name__ == '__main__':
    test_controller()
