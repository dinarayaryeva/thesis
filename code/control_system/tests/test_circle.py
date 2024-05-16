from mj_simulator.bluerov_sim import BluerovSim
from mj_simulator.routines.logger import Logger
from control_system.controllers import SlidingMode, ModelBasedPID
from control_system.quat_routines import euler2quat
from mj_simulator.routines.plotter import plot_trajectory
from time import perf_counter
import numpy as np


# Set initial position for the robot's joints.
q0 = np.array([0, 1.0, 0, 0.9238795, 0, 0, 0.3826834])
v0 = np.array([0, 0, 0, 0, 0, 0])

sim = BluerovSim("../assets/bluerov.xml",
                 update_freq=500,
                 freq=2000,
                 buoyancy = 1.01,
                #  stream = [0.2, 0.1, -0.2],
                 render=True,
                 by_step=True,
                 q0=q0, v0=v0)

t0 = perf_counter()
t = 0
tf = 15.0

# Initialize the control signal array with zeros.
control = np.zeros(sim.nu)

r = 1.0
ts = np.arange(t, tf, 0.001)

controller = SlidingMode(model=sim.model)

logger = Logger(freq=500)
logger.set_labels(['t', 'x', 'v', 'u'])
logger.start()

i = 0
qs = []

try:
    while t < tf:
        t = perf_counter() - t0
        if t >= ts[i]:
            # quat_des = euler2quat(0, 0, -2*np.pi/20*ts[i])
            quat_des = [1, 0, 0, 0]
            q_des = [r*np.sin(2*np.pi/20*ts[i]), r*np.cos(2*np.pi/20*ts[i]),
                     0, quat_des[0], quat_des[1], quat_des[2], quat_des[3]]
            # dq_des = [-2*np.pi/10*r*np.sin(2*np.pi/10*ts[i]), -2*np.pi/10*r*np.sin(2*np.pi/10*ts[i]), 0, 0, 0, 0]
            qs.append(q_des)
            # Get the current joint positions (q) and velocities (dq).
            q, dq = sim.state.q, sim.state.dq
            # Set the control signal to the simulator.
            u = controller.control(q, dq, q_des)
            sim.set_control(u)
            sim.step()

            d = {'t': t, 'x': q, 'v': dq, 'u': u}
            logger.set_data(d)

            i += 1

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

    # plot(t_s, x=xs, v=vs, u=us)
    plot_trajectory(x=xs, x_des=qs)
