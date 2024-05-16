from .quat_routines import quat_error
from config.control_config import heavy_inputs_inv
import numpy as np
import cvxpy as cp
from simple_pid import PID
from .srb_model import SRB


class Controller():
    def __init__(self, model):
        self.model = model

    def control(self, x, dx, x_d):
        raise NotImplementedError

    def force2control(self, f):
        u = heavy_inputs_inv@(f)

        coefs = [1.43147096e-04, -9.93856445e-05, -9.06394750e-03, -
                 1.52520646e-04, 3.14036521e-01, -1.97997024e-02]
        n = len(coefs) - 1
        poly_ctrl = [sum([coefs[i]*u_k**(n - i) for i in range(n)])
                     for u_k in u]

        u = np.clip(poly_ctrl, -1, 1)

        return u


class ModelBasedPID(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.pids = [PID(4, 0.001, 0.5),
                     PID(4, 0.001, 0.5),
                     PID(4, 0.001, 3.5),
                     PID(4, 3.3, 1.5),
                     PID(4, 0.3, 1.5),
                     PID(4, 0.3, 1.0)]
        self.L = 1

    def pd_torque(self, x, dx, x_d):

        res_x = np.zeros(7)  # residual
        res_x[:3] = x[:3]
        res_x[3:] = quat_error(x[3:], x_d[3:])

        e_b = self.srb.J(x).T@res_x  # body-frame error

        tau = np.zeros(6)

        for i in range(3):
            self.pids[i].setpoint = x_d[i]
            tau[i] = self.pids[i](x[i])

        orientation_error = e_b[3:]
        for i in range(3):
            tau[i+3] = self.pids[i+3](orientation_error[i])

        return tau

    def control(self, x, dx, x_d, dx_d=None):
        # residual -?
        a_r = -self.L*np.array(dx)

        # compensate dynamics (maybe add other terms -> makes worse (why-?))
        u_m = self.srb.M@a_r + self.srb.g(x)
        u_pid = self.srb.M@self.pd_torque(x, dx, x_d)

        return self.force2control(u_pid + u_m)


class SlidingMode(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.l = 1
        self.phi = [1.3, 1.3, 1.3, 3.8, 3.8, 3.8]  # error bounds
        self.K = [6.56887255,  5.97797786,  7.92404967, 21.88823654, 15.03985469,
                  17.54285714]  # control gains

    def control(self, x, dx, x_d, dx_d=None):

        res_x = np.zeros(7)  # residual
        res_x[:3] = x[:3] - x_d[:3]
        res_x[3:] = quat_error(x[3:], x_d[3:])

        e_b = self.srb.J(x).T@res_x

        if dx_d is None:
            s = dx + self.l*e_b
        else:
            s = dx - dx_d + self.l*e_b

        a_r = - self.l*e_b  # ds/dt=0

        u_hat = self.srb.M@a_r + \
            self.srb.C(dx)@dx + self.srb.D(dx)@dx + \
            self.srb.g(x)

        s_sign = np.sign(s)

        for i in range(len(s_sign)):
            if abs(s[i]) < self.phi[i]:
                s_sign[i] = s[i]/self.phi[i]

        # added constant for smaller error
        u_s = -5*np.diag(self.K)*self.srb.M@s_sign

        return self.force2control(u_hat + u_s)


class RobustQP(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.l = 1
        self.phi = [1.3, 1.3, 1.3, 3.8, 3.8, 3.8]  # error bounds
        self.K = [6.56887255,  5.97797786,  7.92404967, 21.88823654, 15.03985469,
                  17.54285714]  # control gains

    def control(self, x, dx, x_d, dx_d=None):

        res_x = np.zeros(7)  # residual
        res_x[:3] = x[:3] - x_d[:3]
        res_x[3:] = quat_error(x[3:], x_d[3:])

        e_b = self.srb.J(x).T@res_x

        if dx_d is None:
            s = dx + self.l*e_b
        else:
            s = dx - dx_d + self.l*e_b

        a_r = - self.l*e_b  # ds/dt=0

        a_n = self.srb.M@a_r + \
            self.srb.C(dx)@dx + self.srb.D(dx)@dx + \
            self.srb.g(x)

        R_a = np.eye()
        R_u = np.eye()
        gamma_1 = 1
        gamma_2 = 1

        a_s_prev = np.zeros()
        u_min = -100
        u_max = 100

        # Construct the problem.
        a_s = cp.Variable(a_s)
        u = cp.Variable(u)
        d = cp.Variable(d)

        objective = cp.Minimize(cp.quad_form(a_s, R_a) + cp.quad_form(u, R_u)
                                + gamma_1*cp.power(d, 2)
                                + gamma_2*cp.norm(a_s - a_s_prev, 1))
        # TODO find eta and w
        constraints = [5*s.T@np.diag(self.K)@a_s <= -eta*cp.norm(s, 1) + \
                       cp.norm(s, 1) * cp.norm(w, 1),
                       self.srb.M@(a_s + a_n) +
                       self.srb.C(dx)@dx + self.srb.D(dx)@dx +
                       self.srb.g(x) == u,  # not u, but Bu but how?
                       u_min <= u,
                       u_max >= u]
        prob = cp.Problem(objective, constraints)
        prob.solve()

        return self.force2control(u.value)
