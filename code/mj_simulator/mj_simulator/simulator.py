from typing import Optional, Any
from .render import MujocoRender
from .routines.process_builder import TickerProcess
from .shared_types import SharedState, SharedSensors
import numpy as np
import mujoco as mj


class Simulator(TickerProcess):
    """
    A MuJoCo simulation process that runs a physics simulation for a given robot model.

    This class handles the initialization, state updating, rendering, and control
    of a MuJoCo simulation environment. It provides methods to reset the simulation,
    apply control signals, and retrieve state and sensor information. The simulation
    can be run in real-time or as fast as possible depending on the real_time_factor.
    """

    def __init__(
        self,
        xml_path=None,
        model=None,
        update_freq=500,  # for process itself
        freq=2500,  # for mujoco integration
        real_time_factor=1,
        name="simulator",
        niceness=5,
        q0=None,
        v0=None,
        start=False,
        by_step=False,
        render=False,
    ):
        """
        Initializes the simulator with the given parameters and creates a MuJoCo model and data instance.

        Args:
        - xml_path (str): Path to the MuJoCo robot model XML file.
        - model (MjModel): Preloaded MuJoCo model object.
        - update_freq (int): Frequency at which the process itself should be updated.
        - freq (int): Frequency for MuJoCo physics integration.
        - real_time_factor (float): Factor to scale real-time simulation speed.
        - name (str): Name of the simulator process.
        - niceness (int): Priority of the process.
        - q0 (ndarray): Initial position vector for the simulation.
        - v0 (ndarray): Initial velocity vector for the simulation.
        - start (bool): Whether to start the simulation immediately upon initialization.
        - by_step (bool): If True, initializes without starting the simulation.
        - render (bool): If True, initializes the rendering system.
        """

        TickerProcess.__init__(
            self, freq=update_freq, args=[], name=name, niceness=niceness
        )

        self.xml_path = xml_path

        if xml_path is not None:
            self.model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
        elif model is not None:
            self.model = model
        else:
            raise ValueError("Either xml_path or model must be provided")

        self.data = mj.MjData(self.model)  # MuJoCo data
        self.model.opt.timestep = 1.0 / freq
        self.update_freq = update_freq

        # Determine the joint names in the model
        self.joint_names = {
            joint_id: self.model.joint(joint_id).name
            for joint_id in range(self.model.njnt)
        }

        # Set initial state
        if q0 is None:
            mj.mj_resetData(self.model, self.data)
            q0 = self.data.qpos
        if v0 is None:
            v0 = np.zeros(self.model.nv)

        self.q0, self.v0 = q0, v0

        # Initialize shared state and control structures
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nu = self.model.nu
        self.state = SharedState(self.nq, self.nv, self.nu)

        # Initialize sensors
        sensors_dim = [
            len(self.data.sensor(sensor_id).data)
            for sensor_id in range(self.model.nsensor)
        ]
        sensors_name = [
            self.data.sensor(sensor_id).name for sensor_id in range(self.model.nsensor)
        ]
        self.sensors = SharedSensors(
            self.model.nsensor, sensors_name, sensors_dim)

        self.real_time_factor = real_time_factor
        self.__by_step = by_step

        # Initialize or start the simulation based on by_step flag
        if self.__by_step:
            self.on_init()
        elif start:
            self.start()

        # Initialize renderer if necessary
        if render:
            self.renderer = MujocoRender(self)
            self.renderer.start()
        else:
            self.renderer = None

    def on_init(self) -> None:
        """
        Called to initialize the simulation. This method should set up any necessary
        configurations that are required before starting the simulation loop. It is
        particularly useful when the simulation needs to be prepared without immediately
        starting the ticking process.
        """
        self.reset()
        self.update_state()
        self.state._integrator_time[0] = self.data.time

    def reset(self) -> None:
        """
        Reset the simulation to its initial state or to the specified state. This involves
        setting the initial pose according to `self.q0` and `self.v0`.
        """
        # Set initial pose
        mj.mj_resetData(self.model, self.data)
        self.data.qpos = self.q0
        self.data.qvel = self.v0

    def get_state(self) -> SharedState:
        """
        Get the current simulation state.

        Returns:
            SharedState: An object representing the current shared state of the simulation.
        """
        return self.state

    def get_sensors(self) -> SharedSensors:
        """
        Get the current readings from the simulation's sensors.

        Returns:
            SharedSensors: An object representing the current shared sensor readings.
        """
        return self.sensors

    def set_control(self, u: np.ndarray) -> None:
        """
        Apply a control signal to the simulation.

        Parameters:
            u (np.ndarray): The control signal to apply, compatible with the simulation's control array.
        """
        self.state._control[:] = u

    def set_init_state(
        self, q0: Optional[np.ndarray] = None, v0: Optional[np.ndarray] = None
    ) -> None:
        """
        Sets the initial state for the simulation to the specified position and velocity vectors,
        and then resets the simulation to apply these initial conditions.

        Parameters:
            q0 (Optional[np.ndarray]): The initial position vector to set for the simulation.
                                    If None, the current initial position vector is retained.
            v0 (Optional[np.ndarray]): The initial velocity vector to set for the simulation.
                                    If None, the initial velocity vector is set to zeros if
                                    `q0` is provided, otherwise it retains the current initial
                                    velocity vector.
        """
        if q0 is not None:
            self.q0 = q0
            self.v0 = np.zeros(self.model.nv)

        if v0 is not None:
            self.v0 = v0

        self.reset()

    def update_state(self) -> None:
        """
        Update the simulation's internal representation of the state according to the
        convention being used (`pin` or default).
        """
        self.state._q[:] = self.data.qpos.copy()
        self.state._dq[:] = self.data.qvel.copy()

    def update_sensors(self) -> None:
        """
        Update the simulation's internal representation of the sensor readings by copying
        the latest sensor data.
        """
        self.sensors._values[:] = self.data.sensordata

    def on_while(self) -> None:
        """
        Execute the simulation steps in a loop until the simulation time reaches the
        specified limit based on the real-time factor.
        """
        while self.data.time <= self.time * self.real_time_factor:
            mj.mj_step(self.model, self.data)

    def step(self) -> None:
        """
        Perform a single simulation step, increment the internal time, execute the
        simulation loop, and trigger the target update.
        """
        self.time += 1 / self.update_freq
        self.on_while()
        self.target()

    def target(self) -> None:
        """
        Set the target state and sensor readings based on the current simulation data
        and control signals.
        """
        self.state._integrator_time[0] = self.data.time
        self.state._time[0] = self.time
        self.data.ctrl[:] = self.state._control[:]
        self.update_state()
        self.update_sensors()

    def close(self) -> None:
        """
        Close the simulation environment, and rendering engine
        being used.
        """
        if not self.__by_step:
            super().close()
        if self.renderer is not None:
            self.renderer.close()
