from .quat_routines import quat_error, so3_error
from config.control_config import heavy_inputs_inv
import numpy as np
from simple_pid import PID
from .srb_model import SRB


class Controller():
    def __init__(self, model):
        self.model = model

    def control(self, x, dx, x_d):
        raise NotImplementedError


class ModelBasedPID(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.pids = [PID(1, 0.001, 0.0),
                     PID(1, 0.001, 0.0),
                     PID(1.2, 0.001, 0.0)]

        self.p_gain = np.diag([3, 2, 5])
        self.d_gain = np.diag([2, 1, 3])
        self.L = 1

    def pd_torque(self, x, dx, x_d):

        tau = []

        for i in range(3):
            tau.append(self.pids[i](x[i]))

        orientation_error = so3_error(x[3:], x_d[3:])
        omega = np.array(dx[3:])
        body_torque = self.p_gain @ orientation_error - self.d_gain @ omega
        tau.extend(body_torque)

        return tau

    def control(self, x, dx, x_d):

        e_b = self.srb.J(x).T@(np.array(x) - np.array(x_d))
        v_r = self.L*e_b
        a_r = -self.L*np.array(dx)  # or v_r ???

        u_m = self.srb.M@a_r + self.srb.g(x)
        u_pid = self.srb.M@self.pd_torque(x, dx, x_d)

        return heavy_inputs_inv@(u_pid + u_m)


class SlidingMode(Controller):
    def __init__(self, model):
        super().__init__(model)

        self.srb = SRB(model)
        self.l = 1
        self.phi = [0.05, 0.05, 0.05, 0.3, 0.3, 0.3]  # error bounds
        self.K = [6.56887255,  5.97797786,  7.92404967, 21.88823654, 15.03985469,
                  17.54285714]  # control gains

    def control(self, x, dx, x_d):

        res_x = np.zeros(7)  # residual
        res_x[:3] = x[:3] - x_d[:3]
        res_x[3:] = quat_error(x[3:], x_d[3:])

        e_b = self.srb.J(x).T@res_x
        s = dx + self.l*e_b
        a_r = - self.l*e_b  # ds/dt=0

        u_hat = self.srb.M@a_r + \
            self.srb.C(dx)@dx + self.srb.D(dx)@dx + \
            self.srb.g(x) - self.l*e_b

        s_sign = np.sign(s)

        for i in range(len(s_sign)):
            if abs(s[i]) < self.phi[i]:
                s_sign[i] = s[i]/self.phi[i]

        u_s = -np.diag(self.K)*self.srb.M@s_sign

        return heavy_inputs_inv@(u_hat + u_s)
