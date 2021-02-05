import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt


class Quaternion:
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

    def as_array(self):
        return np.array([self.q_0, self.q_1, self.q_2, self. q_3])

    def as_matrix(self):
        return Rotation.from_quat(self.as_array()).as_matrix()

    def normalize(self):
        arr = self.as_array()
        return Quaternion.from_array(arr / np.linalg.norm(arr))

    def as_rpy(self):
        rpy = Rotation.from_quat(self.as_array()).as_euler('ZYX')[::-1]
        rpy[1] *= -1
        return rpy

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


def main():
    orientation = Quaternion()
    velocity = Quaternion(q_3=0)



if __name__ == '__main__':
    main()
