# An introductory example showing a simple simulation.

from mj_simulator.simulator import Simulator
from time import perf_counter

# Creating an instance of Simulator with various parameters:
sim = Simulator("../assets/bluerov.xml",         # Path to the MuJoCo XML configuration file.
                update_freq=500,  # Update frequency for the control and state (Hz).
                freq=2000,        # Simulation step frequency (Hz).
                render=True,    # Enable graphical rendering of the simulation.
                start=True)       # Start the simulation upon instantiation.

# Initialize a timer for the process using perf_counter,
# which provides a high resolution timer.
t0 = perf_counter()
t = 0
tf = 10.0  # Total simulation time (60 seconds).
ts = 0

try:
    # Start a loop that will run for 60 seconds (tf).
    while t < tf:
        # Update the current time elapsed.
        t = perf_counter() - t0

        # Check if at least 0.01 seconds have passed since the last update.
        if t - ts >= 0.01:
            # Placeholder for code that would run every 0.01 seconds.
            # For example, this could be where
            # the simulation state is printed or logged.
            #
            ts = t

    # Close the simulator once the while loop is done.
    sim.close()

# If a KeyboardInterrupt (Ctrl+C) is detected, close the simulator gracefully.
except KeyboardInterrupt:
    # Handle any cleanup if the simulation is interrupted.
    print("Simulation interrupted.")
finally:
    # Perform any finalization steps, such as closing the simulator window.
    print("Finalizing simulation...")
    sim.close()
