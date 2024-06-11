from mj_simulator.bluerov_sim import BluerovSim
from mj_simulator.routines.logger import Logger
from control_system.controllers import SlidingMode, ModelBasedPID, RobustQP
from mj_simulator.routines.plotter import plot_norm
from time import perf_counter
import numpy as np

# Set initial position for the robot's joints.
q0 = np.array([0.5, -0.3, -0.25, 0.9238795, 0, 0, 0.3826834])
v0 = np.array([0, 0, 0, 0, 0, 0])
sim = BluerovSim("../assets/bluerov.xml",
                update_freq=500,
                freq=2000,
                buoyancy = 1.05,
                stream = [0.2, -0.3, 0.1],
                # voltage=14,
                # render=True,
                by_step=True,
                q0=q0, v0=v0)

t0 = perf_counter()
t = 0
tf = 11.0
ts = 0

# Initialize the control signal array with zeros.
control = np.zeros(sim.nu)

# Set the desired position for the robot's joints.
q_des = np.array([0, 0, 0, 1, 0, 0, 0])

controller = ModelBasedPID(model=sim.model)

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

    t_s_id = logger.get_storage()["t"]
    xs_id = logger.get_storage()["x"]
    vs_id = logger.get_storage()["v"]
    us_id = logger.get_storage()["u"]

    logger.close()

sim = BluerovSim("../assets/bluerov.xml",
                update_freq=500,
                freq=2000,
                buoyancy = 1.01,
                stream = [0.1, -0.1, 0.1],
                # voltage=14,
                # render=True,
                by_step=True,
                q0=q0, v0=v0)

t0 = perf_counter()
t = 0
tf = 11.0
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

    t_s_sm = logger.get_storage()["t"]
    xs_sm = logger.get_storage()["x"]
    vs_sm = logger.get_storage()["v"]
    us_sm = logger.get_storage()["u"]

    logger.close()

sim = BluerovSim("../assets/bluerov.xml",
                update_freq=500,
                freq=2000,
                buoyancy = 1.01,
                stream = [0.1, -0.1, 0.1],
                # voltage=14,
                # render=True,
                by_step=True,
                q0=q0, v0=v0)

t0 = perf_counter()
t = 0
tf = 11.0
ts = 0

# Initialize the control signal array with zeros.
control = np.zeros(sim.nu)

# Set the desired position for the robot's joints.
q_des = np.array([0, 0, 0, 1, 0, 0, 0])

controller = RobustQP(model=sim.model)

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

    t_s_qp = logger.get_storage()["t"]
    xs_qp = logger.get_storage()["x"]
    vs_qp = logger.get_storage()["v"]
    us_qp = logger.get_storage()["u"]

    logger.close()

# plot_norm([t_s_id, xs_id], [t_s_sm, xs_sm], [t_s_qp, xs_qp])
plot_norm([t_s_id, us_id], [t_s_sm, us_sm], [t_s_qp, us_qp])