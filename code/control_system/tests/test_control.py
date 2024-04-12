from mj_simulator.bluerov_sim import BluerovSim
from mj_simulator.routines.logger import Logger
from control_system.controllers import SlidingMode
from mj_simulator.routines.plotter import plot
from time import perf_counter
import numpy as np

# Set initial position for the robot's joints.
q0 = np.array([1, -1, -1.5, 0.9238795, 0, 0, 0.3826834])
v0 = np.array([1, 0, 0, 0, 0, 0])

sim = BluerovSim("../assets/bluerov.xml",
                update_freq=500,
                freq=2000,
                buoyancy = 1.01,
                stream = [1.0, 0, 0],
                render=True,
                by_step=True,
                q0=q0, v0=v0)

t0 = perf_counter()
t = 0
tf = 10.0
ts = 0

# Initialize the control signal array with zeros.
control = np.zeros(sim.nu)

# Set the desired position for the robot's joints.
q_des = np.array([0, 0, 0, 1, 0, 0, 0])

controller = SlidingMode(model=sim.model)

logger = Logger(freq=500)
logger.set_labels(['t', 'x', 'v', 'u'])
logger.start()

try:
    while t < tf:
        t = perf_counter() - t0

        if t - ts >= 0.002:
            # Get the current joint positions (q) and velocities (dq).
            q, dq = sim.state.q, sim.state.dq
            # Set the control signal to the simulator.
            u = controller.control(q, dq, q_des)
            sim.set_control(u)
            sim.step()

            d = {'t': t, 'x': q, 'v': dq, 'u': u}
            logger.set_data(d)

            ts = t

except KeyboardInterrupt:
    # Handle any cleanup if the simulation is interrupted.
    print("Simulation interrupted.")
finally:
    # Perform any finalization steps, such as closing the simulator window.
    print("Finalizing simulation...")
    sim.close()

    t_s = logger.get_storage()["t"]
    xs = logger.get_storage()["x"]
    vs = logger.get_storage()["v"]
    us = logger.get_storage()["u"]

    logger.close()

    plot(t_s, x=xs, v=vs, u=us, separate=True)
