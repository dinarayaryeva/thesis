from .simulator import Simulator
import numpy as np
import mujoco as mj


class BluerovSim(Simulator):
    def __init__(self,
                 xml_path,
                 update_freq=500,    # for process itself
                 freq=2500,          # for mujoco integration
                 name="bluerov_sim",
                 buoyancy=1.01,      # buoyancy coefficient
                 voltage=20,         # power cell voltage
                 stream=None,        # stream xyz values
                 q0=None,
                 v0=None,
                 start=False,
                 by_step=False,
                 render=False):

        model = mj.MjModel.from_xml_path(xml_path)

        # set buoyancy force
        mass = model.body('body').mass[0]
        W = -mass*(model.opt.gravity[-1])
        self.B = W*buoyancy

        # set stream velocity
        model.opt.wind = stream if stream else [0, 0, 0]

        # set pwm coefficient
        voltage = np.clip(voltage, 10, 20)
        self.k = np.interp(voltage, [10, 20], [0.4, 1])

        super().__init__(model=model, update_freq=update_freq, freq=freq,
                         name=name, q0=q0, v0=v0, start=start, by_step=by_step, render=render)

    def reset(self) -> None:
        """
        Reset the simulation to its initial state or to the specified state. This involves
        setting the initial pose according to `self.q0` and `self.v0`.
        """
        # Set initial pose
        mj.mj_resetData(self.model, self.data)
        self.data.qpos = self.q0
        self.data.qvel = self.v0

        # set buoyancy force
        self.data.xfrc_applied[self.model.body('buoyancy').id][2] = self.B

    def set_control(self, u: np.ndarray) -> None:
        """
        Apply a control signal to the simulation.

        Parameters:
            u (np.ndarray): The control signal to apply, compatible with the simulation's control array.
        """
        coefs = [3.68121674, 0.83909022, 2.54794587, -0.01861895]
        n = len(coefs) - 1

        poly_ctrl = np.array([sum([coefs[i]*u_k**(n - i) for i in range(n)])
                              for u_k in u])
        f = self.k * poly_ctrl
        self.state._control[:] = f
        # self.state._control[:] = u
