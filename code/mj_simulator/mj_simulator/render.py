from .routines.process_builder import TickerProcess
from .shared_types import SharedState
import mujoco as mj
from mujoco import viewer
import numpy as np


class MujocoRender(TickerProcess):
    """Handles the rendering process for MuJoCo simulations.

    This class extends TickerProcess to create a rendering loop that updates
    and renders a MuJoCo simulation at a specified frequency.

    Attributes:
        simulator (mujoco.MjSim): The MuJoCo simulator instance to render.
        model (mujoco.MjModel): The MuJoCo model taken from the simulator.
        data (mujoco.MjData): The simulation data updated for rendering.
        __convention (str): The naming convention used by the simulator.
        viewer (mujoco.viewer.MjViewer): The MuJoCo viewer for rendering.
    """

    def __init__(
        self,
        simulator=None,
        model=None,
        freq=60,
        name="render",
        niceness=15,
        start=False,
    ):
        """
        Initializes the renderer process for MuJoCo simulations.

        Args:
            simulator (mujoco.MjSim): An instance of the MuJoCo simulator.
            model (MjModel): Preloaded MuJoCo model object.
            freq (int): The frequency at which the simulation is rendered.
            name (str): The name of the process.
            niceness (int): The niceness setting for the process scheduling priority.
            convention (str): The convention used for representing the state, "pin" or "mj".
            start (bool): Whether to start the simulation immediately upon initialization.
        """
        TickerProcess.__init__(self, freq=freq, args=[],
                               name=name, niceness=niceness)

        if simulator is not None:
            self.simulator = simulator
            self.model = simulator.model  # MuJoCo model
            self.data = mj.MjData(self.model)  # MuJoCo data

            self.state = simulator.state

        elif model is not None:
            self.simulator = None
            self.model = model  # MuJoCo model
            self.data = mj.MjData(self.model)  # MuJoCo data

            self.state = SharedState(
                self.model.nq, self.model.nv, self.model.nu)
        else:
            raise ValueError("Either simulator or model must be provided")

        if start:
            self.start()

    def update_state(self):
        """Updates the MuJoCo simulation data with the current renderer state."""
        pos, vel = self.state.q, self.state.dq

        # Update the MuJoCo data with the new position, velocity, and control inputs
        self.data.qpos = pos
        self.data.qvel = vel
        self.data.ctrl = self.state._control[:]
        # Run the forward dynamics to reflect the updated state in the data
        mj.mj_forward(self.model, self.data)

    def set_state(self, q: np.array, v: np.array = None, u: np.ndarray = None) -> None:
        """
        Updates the internal state of the renderer.

        Args:
            q (np.ndarray): The configuration vector.
            v (np.ndarray): The velocity vector.
            u (np.ndarray): The control signal to apply.
        """

        if v is None:
            v = np.zeros(self.model.nv)
        if u is None:
            u = np.zeros(self.model.nu)

        self.state._q[:] = q
        self.state._dq[:] = v
        self.state._control[:] = u

    def setup_viewer(self):
        """Sets up the MuJoCo viewer for rendering the simulation."""
        # Launch a passive viewer with specific UI elements hidden and a custom key callback
        self.viewer = viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
            key_callback=self.key_callback,
        )
        # Enable visualization of contact points and forces
        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        # Set the visualization scale for forces
        self.model.vis.map.force = 0.005

    def on_init(self):
        """Initial setup function that is called when the rendering process starts."""
        # Configure the viewer for rendering
        self.setup_viewer()
        self.reset()

    # FIXME - fix process pause
    def key_callback(self, keycode):
        """Keyboard callback to handle user input during the rendering process.

        Args:
            keycode (int): The ASCII code of the key pressed.
        """
        if (
            chr(keycode) == " "
        ):  # If spacebar is pressed, pause or resume the simulation
            if self._shared.is_active:
                self.pause()
            else:
                self.resume()
        elif chr(keycode) == "R":  # If 'R' is pressed, reset the simulation
            self.reset()

    def reset(self):
        """Resets the simulation to its initial state and updates the viewer."""

        mj.mj_resetData(self.model, self.data)
        self.set_state(self.data.qpos)

        # Resets the simulator to a random initial state
        if self.simulator is not None:
            self.simulator.reset()

        # Update the simulation state in the data
        self.update_state()
        # Sync the viewer with the updated state
        self.viewer.sync()

    def target(self):
        """The main rendering loop that is called at each tick to update the viewer."""
        # Update the simulation state with the latest from the simulator
        self.update_state()
        self.state._time[0] = self.time
        # Sync the viewer with the updated state
        self.viewer.sync()
