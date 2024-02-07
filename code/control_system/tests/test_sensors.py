# This script reads sensor values from a robot model during the simulation.

from mj_simulator.bluerov_sim import BluerovSim
from time import perf_counter


sim = BluerovSim("../assets/bluerov.xml", update_freq=500, freq=2000, render=False, start=True)

t0 = perf_counter()
t = 0
tf = 1.0
ts = 0

try:
    while t < tf:
        t = perf_counter() - t0

        # Check if at least 0.01 seconds have passed since the last update.
        if t - ts >= 0.01:
            # to get all sensors
            print("\nSensors values:", sim.sensors.values)
            # to get specific sensor by its name from xml
            print("Sensors data:")
            for sensor_name in sim.sensors.names:
                print(sensor_name, getattr(sim.sensors, sensor_name))
            ts = t

    # Close the simulator once the while loop is done.
    sim.close()

except KeyboardInterrupt:
    # Handle any cleanup if the simulation is interrupted.
    print("Simulation interrupted.")
finally:
    # Perform any finalization steps, such as closing the simulator window.
    print("Finalizing simulation...")
    sim.close()
