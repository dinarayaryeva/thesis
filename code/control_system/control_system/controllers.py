from .quat_routines import quat_error
from config.control_config import heavy_inputs_inv, heavy_mapping
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
        # u = heavy_inputs_inv@(f)
        u = f

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


# class SlidingMode(Controller):
#     def __init__(self, model):
#         super().__init__(model)

#         self.srb = SRB(model)
#         self.l = 1
#         self.phi = [1.3, 1.3, 1.3, 3.8, 3.8, 3.8]  # error bounds
#         self.K = [6.56887255,  5.97797786,  7.92404967, 21.88823654, 15.03985469,
#                   17.54285714]  # control gains

#     def control(self, x, dx, x_d, dx_d=None):

#         res_x = np.zeros(7)  # residual
#         res_x[:3] = x[:3] - x_d[:3]
#         res_x[3:] = quat_error(x[3:], x_d[3:])

#         e_b = self.srb.J(x).T@res_x

#         if dx_d is None:
#             s = dx + self.l*e_b
#         else:
#             s = dx - dx_d + self.l*e_b

#         a_r = - self.l*e_b  # ds/dt=0

#         u_hat = self.srb.M@a_r + \
#             self.srb.C(dx)@dx + self.srb.D(dx)@dx + \
#             self.srb.g(x)

#         s_sign = np.sign(s)

#         for i in range(len(s_sign)):
#             if abs(s[i]) < self.phi[i]:
#                 s_sign[i] = s[i]/self.phi[i]

#         # added constant for smaller error
#         u_s = -5*np.diag(self.K)*self.srb.M@s_sign

#         return self.force2control(u_hat + u_s)
    
class SlidingMode(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.l = 1
        self.sigma = 1.5
        self.eps = [0.05, 0.05, 0.05, 0.8, 0.8, 0.8]  # error bounds
        self.alpha = 93

    def control(self, x, dx, x_d, dx_d=None):

        res_x = np.zeros(7)  # residual
        res_x[:3] = x[:3] - x_d[:3]
        res_x[3:] = quat_error(x[3:], x_d[3:])
        r_tilde = self.srb.J(x).T@res_x  # in body frame
        v_tilde = dx
        if dx_d is not None:
            v_tilde = dx - dx_d

        # p-regulator part -> pd or pid
        a_n = -self.l*r_tilde

        u_n = self.srb.M@a_n + \
            self.srb.C(dx)@dx + self.srb.D(dx)@dx + \
            self.srb.g(x)
        
        # sliding part
        s = v_tilde + self.l*r_tilde
        s_sign = np.sign(s)

        for i in range(len(s_sign)):
            if abs(s[i]) < self.eps[i]:
                s_sign[i] = s[i]/self.eps[i]

        rho = self.alpha/(self.sigma)**2*self.srb.M_inv
        a_s = rho@s_sign
        u_s = -self.srb.M@a_s

        return self.force2control(u_n + u_s)


class RobustQP(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.l = 1

    def control(self, x, dx, x_d, dx_d=None):

        res_x = np.zeros(7)  # residual
        res_x[:3] = x[:3] - x_d[:3]
        res_x[3:] = quat_error(x[3:], x_d[3:])
        r_tilde = self.srb.J(x).T@res_x  # in body frame
        v_tilde = dx
        if dx_d is not None:
            v_tilde = dx - dx_d

        # p-regulator part -> pd or pid
        a_n = -self.l*r_tilde
        
        # sliding part
        s = v_tilde + self.l*r_tilde

        R_a = np.eye(6)
        R_u = np.eye(8)
        gamma_1 = 5

        u_min = -5.4
        u_max = 7
        w = 37
        K = np.diag([0.36]*6)

        # Construct the problem.
        a_s = cp.Variable(6)
        u = cp.Variable(8)
        d = cp.Variable(1)

        objective = cp.Minimize(cp.quad_form(a_s, R_a) + cp.quad_form(u, R_u)
                                + gamma_1*cp.power(d, 2))
        
        constraints = [-s.T@K@a_s >= -5*cp.norm(s, 1) - \
                       cp.norm(s, 1) * w + d,
                       self.srb.M@(a_s + a_n) +
                       self.srb.C(dx)@dx + self.srb.D(dx)@dx +
                       self.srb.g(x) == heavy_mapping@u,
                       u_min <= u,
                       u_max >= u]
        
        prob = cp.Problem(objective, constraints)
        prob.solve()

        return self.force2control(u.value)
