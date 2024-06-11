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

    def force2control(self, u):

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
        self.pids = [PID(4, 0.001),
                     PID(4, 0.001),
                     PID(4, 0.001),
                     PID(4, 3.3),
                     PID(4, 0.3),
                     PID(4, 0.3)]
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

        return self.force2control(heavy_inputs_inv@(u_pid + u_m))


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
        self.eps = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]  # error bounds
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

        return self.force2control(heavy_inputs_inv@(u_n + u_s))


class RobustQP(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.l = 1
        self.prob = self.build_problem()
        self.a_s_prev = np.zeros(6)

    def build_problem(self):
        
        # tuning
        R_a = np.eye(6)
        R_u = 0.01*np.eye(8)
        gamma_1 = 0.1
        # gamma_2 = 0.2

        # approximated max bound
        u_min = -5.4
        u_max = 7
        w = 37
        K = np.diag([15.0]*6)

        # state parameters
        C_dx = cp.Parameter((6, ), name = 'C_dx')
        D_dx = cp.Parameter((6, ), name = 'D_dx')
        g = cp.Parameter((6, ), name = 'g')
        s = cp.Parameter((6, ), name = 's')
        a_n = cp.Parameter((6, ), name = 'a_n')
        a_s_prev = cp.Parameter((6, ), name = 'a_s_prev')

        # problem variables
        a_s = cp.Variable((6, ), name = 'a_s')
        u = cp.Variable((8, ), name = 'u')
        d = cp.Variable(1, name = 'd')

        objective = cp.Minimize(cp.sum_squares(R_a @ a_s) + cp.sum_squares(R_u @ u) + gamma_1*cp.power(d, 2))
        
        constraints = [s.T@K@a_s >= 100*cp.norm(s) + \
                       cp.norm(s) * w + d,
                       self.srb.M@(a_s + a_n) + \
                       C_dx + D_dx + \
                       g == heavy_mapping@u,
                       u_min <= u,
                       u_max >= u]
        
        return cp.Problem(objective, constraints)

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

        self.prob.param_dict['C_dx'].value = self.srb.C(dx)@dx
        self.prob.param_dict['D_dx'].value = self.srb.D(dx)@dx
        self.prob.param_dict['g'].value = self.srb.g(x)

        self.prob.param_dict['s'].value = -s
        self.prob.param_dict['a_n'].value = a_n
        # self.prob.param_dict['a_s_prev'].value = self.a_s_prev

        self.prob.solve(warm_start=True)

        u = self.prob.var_dict['u'].value
        self.a_s_prev = self.prob.var_dict['a_s'].value

        return self.force2control(u)
