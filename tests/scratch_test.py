import numpy as np

from rocket_landing.params import VehicleParams
from rocket_landing.controllers.pid import PIDController
from rocket_landing.simulation import simulate_pid

p = VehicleParams()

pid_vz = PIDController(kp=1500,ki=50.0, kd=800.0) # same as step 25
pid_x = PIDController(kp=500.0, ki=0.0, kd=2000.0)
pid_y = PIDController(kp=500.0, ki=0.0, kd=2000.0)

# Start 100 m off in x AND 50 m off in y, so both horizontal PIDs
# (pid_x and pid_y) actually have an error to correct - same vertical
# fall as step 25.
state0 = np.array([100.0, 50.0, 500.0, 0.0, 0.0, -50.0, p.initial_mass])

times, states = simulate_pid(state0, pid_vz, pid_x, pid_y, p, dt=0.05)

print("launched after: ", times[-1], "s")
print("touchdown x: ", states[-1][0])
print("touchdown y: ", states[-1][1])
print("touchdown height: ", states[-1][2])
print("touchdown vz: ", states[-1][5])
print("touchdown mass: ", states[-1][6])