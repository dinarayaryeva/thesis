from multiprocessing import Array
import numpy as np


class SharedState:
    """Represents a shared state object with time, integrator time, and position and velocity arrays.

    Attributes:
        time (numpy.ndarray): A shared NumPy array representing time.
        integrator_time (numpy.ndarray): A shared NumPy array representing integrator time.
        q (numpy.ndarray): A shared NumPy array representing position.
        dq (numpy.ndarray): A shared NumPy array representing velocity.
        control (numpy.ndarray): A shared NumPy array representing torques.

    Args:
        nq (int): The number of elements in the position array.
        nv (int): The number of elements in the velocity array.
        nu (int): The number of elements in the torques array.
    """

    def __init__(self, nq: int, nv: int, nu: int):
        # Internal multiprocessing arrays for shared memory
        self._time = Array('d', [0])
        self._integrator_time = Array('d', [0])
        self._q = Array('d', nq * [0])
        self._dq = Array('d', nv * [0])
        self._control = Array("d", [0] * nu)

        # NumPy arrays as views onto the shared memory
        self.time = np.frombuffer(self._time.get_obj(), dtype=np.double)
        self.integrator_time = np.frombuffer(self._integrator_time.get_obj(), dtype=np.double)
        self.q = np.frombuffer(self._q.get_obj(), dtype=np.double)
        self.dq = np.frombuffer(self._dq.get_obj(), dtype=np.double)
        self.control = np.frombuffer(self._control.get_obj(), dtype=np.double)


class SharedSensors:
    """Represents a shared sensors object with named sensor data arrays.

    Attributes:
        values (numpy.ndarray): A shared NumPy array representing sensor data.
        names (List[str]): A list of names for each sensor.

    Args:
        ns (int): The number of sensors.
        names (List[str]): The names of the sensors.
        dims (List[int]): The dimensionality of the sensors arrays.
    """


    def __init__(self, ns: int, names: list, dims: list):
        # Validate dimensions
        if ns != len(names) or ns != len(dims):
            raise ValueError("The length of names and dims must match the number of sensors (ns).")

        self._values = Array("d", sum(dims) * [0])
        self.values = np.frombuffer(self._values.get_obj(), dtype=np.double)
        self.names = names

        offset = 0
        for i in range(ns):
            setattr(self, names[i], self.values[offset : offset + dims[i]])
            offset += dims[i]
