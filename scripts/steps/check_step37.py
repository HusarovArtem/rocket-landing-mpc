import numpy as np

from rocket_landing.controllers.mpc import build_mpc_nlp, add_state_and_actuator_constraints, MpcWeights, \
    solve_open_loop_trajectory, simulate_mpc
from rocket_landing.params import VehicleParams
from rocket_landing.reference import LandingTarget


params = VehicleParams()
weights = MpcWeights()
target = LandingTarget()


state0 = np.array([100.0, 50.0, 500.0, 0.0, 0.0, -50.0, 25000.0])
N = 30
dt = 0.1
max_steps=8



_, states_cold, _, iters_cold = simulate_mpc(
    state0, params, weights, target, N, dt, max_steps=max_steps, warm_start=False
)
_, states_warm, _, iters_warm = simulate_mpc(
    state0, params, weights, target, N, dt, max_steps=max_steps, warm_start=True
)

print("iter_counts (cold/no warm-start):", iters_cold)
print("iter_counts (warm-start):", iters_warm)
print("\ntotal iterations, step 0 (should match - no prior solution either way):",
      iters_cold[0], "vs", iters_warm[0])
print("total iterations, step 1...end:   ",
      sum(iters_cold[1:]), "(cold) vs", sum(iters_warm[1:]), "(warm)")

final_diff = np.max(np.abs(states_cold[-1] - states_warm[-1]))
print("\nmax diff between states cold vs warm):", final_diff)