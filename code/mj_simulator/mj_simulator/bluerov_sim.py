from .simulator import Simulator
import numpy as np
import mujoco as mj

class BluerovSim(Simulator):
    def __init__(self,
                 xml_path,
                 update_freq=500,  # for process itself
                 freq=2500,  # for mujoco integration
                 name="bluerov_sim",
                 buoyancy=1.01,  # buoyancy coefficient
                 stream = None,
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

    # TODO check poly coeff 
    def set_control(self, u: np.ndarray) -> None:
        """
        Apply a control signal to the simulation.

        Parameters:
            u (np.ndarray): The control signal to apply, compatible with the simulation's control array.
        """
        mu = 0
        std = 33
        u_norm = (u-mu)/std
        coefs = [0.054608309216589, 0.0043623874523217, -0.320974346473973, -0.0639160871326408, 1.38048196118199, 0.0635708584715591]
        poly_ctrl = [sum([coefs[i]*u_k**(5 - i) for i in range(6)]) for u_k in u_norm]
        u_f = np.array(poly_ctrl)*std + mu
        self.state._control[:] = u_f
        # self.state._control[:] = u