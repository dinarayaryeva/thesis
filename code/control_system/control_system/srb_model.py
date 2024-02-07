import mujoco as mj
import numpy as np
import sympy as sp
from sympy import lambdify
from .sympy_routines import quat2rot, skew, quaternion_product


class SRB():
    def __init__(self, model):
        # fetch parameters
        self.m = model.body('body').mass[0]
        self.I_0 = np.diag(model.body('body').inertia)
        self.r_b = model.body('buoyancy').pos

        # inertia
        self.M = np.block([[self.m*np.eye(3), np.zeros((3, 3))],
                     [np.zeros((3, 3)), self.I_0]])
        self.M_inv = np.linalg.inv(self.M)

        self.W = -self.m*(model.opt.gravity[-1])  # weight
        self.B = self.W*1.0  # buoyancy

        # коэффициенты взяты из головы
        self.K_1 = np.diag([10, 10, 10, 0, 0, 0])
        self.K_2 = np.diag([0.1, 0.1, 0.1, 0, 0, 0])

        # reduce computation time
        x = sp.symbols('x0:7')
        v = sp.symbols('v0:6')
        self.J = lambdify([x], self.__jacobian(x[3:]))
        self.R = lambdify([x], quat2rot(x[3:]).T)
        self.T = lambdify([x], quaternion_product(x[3:]))
        self.C = lambdify([v], self.__coriolis(v[3:]))

    def __jacobian(self, q):
        return sp.BlockMatrix([[quat2rot(q).T, sp.zeros(3, 3)], [sp.zeros(4, 3), quaternion_product(q)]])

    def __coriolis(self, w):
        return sp.BlockMatrix([[self.m*skew(w), sp.zeros(3, 3)], [sp.zeros(3, 3), -skew(self.I_0@w)]])

    def D(self, dx):
        speed = np.linalg.norm(dx)

        damping_lin = self.K_1
        damping_quad = self.K_2*speed

        return damping_lin + damping_quad

    def g(self, x):
        g_ = np.zeros((6, ))

        fg = -self.R(x).T@np.array([0, 0, self.W])
        fb = self.R(x).T@np.array([0, 0, self.B])

        g_[:3] = fg + fb
        g_[3:] = np.cross(self.r_b, fb)
        return g_
