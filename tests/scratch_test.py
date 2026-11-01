import numpy as np

from rocket_landing.params import VehicleParams
from rocket_landing.controllers.pid import PIDController
from rocket_landing.simulation import simulate_vertical_pid


p = VehicleParams()
pid = PIDController(kp=1400.0, ki=100.0, kd=2000.0)

# Same starting point at step 21: at rest, 500 m up
state0 = np.array([0.0, 0.0, 500.0, 0.0, 0.0, -50.0, p.initial_mass])

times, states = simulate_vertical_pid(state0, pid, p, dt=0.05)

print("landed after:", times[-1], "s")
print("touchdown height:", states[-1][2])
print("touchdown vz:", states[-1][5])
print("touchdown mass:", states[-1][6])