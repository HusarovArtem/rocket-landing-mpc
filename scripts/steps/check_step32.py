import numpy as np

from rocket_landing.controllers.mpc import MpcWeights, stage_cost, terminal_cost
from rocket_landing.reference import LandingTarget

target = LandingTarget() # position=[0,0,0], velocity[0,0,0]
weights = MpcWeights()

# --- Test 1: stage_cost ------------------------------------------------
# state: only position (indices 0-2) and velocity (3-5) matter here, mass
# (index 6) is irrelevant for state_cost, so it's set to an arbitrary value.
state_k = np.array([10.0, -5.0, 100.0, 1.0, -2.0, 0.5, 24000.0])
control_k = np.array([200000.0, 0.05, -0.02])
control_prev =np.array([190000.0, 0.04, -0.01])

cost_k = stage_cost(state_k, control_k, control_prev, target, weights)
print("stage_cost", float(cost_k))


# -- Test 2: terminal cost ---------------------------------------------
# state_N: close to the pad, small leftover velocity, some fuel burned.
state_N = np.array([2.0, -1.0, 0.0, 0.5, -0.3, -0.8, 23000.0])
initial_mass = 25000.0

cost_N = terminal_cost(state_N, initial_mass, target, weights)
print("terminal_cost:", float(cost_N))