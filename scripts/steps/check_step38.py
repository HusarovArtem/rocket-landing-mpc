import logging
import numpy as np

from rocket_landing.controllers.mpc import build_mpc_nlp, add_state_and_actuator_constraints, MpcWeights, \
    solve_open_loop_trajectory, simulate_mpc
from rocket_landing.params import VehicleParams
from rocket_landing.reference import LandingTarget

logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")

params = VehicleParams()
weights = MpcWeights()
target = LandingTarget()
N = 30
dt = 0.1

print("--- Test 1: normal scenario, expect NO failures ---")
state0_ok = np.array([100.0, 50.0, 500.0, 0.0, 0.0, -50.0, 25000.0])
_, _, controls_ok, _, failed_ok, = simulate_mpc(
    state0_ok, params, weights, target, N, dt, max_steps=5
)
print("solve_failed:", failed_ok)

print("--- Test 2: infeasible-from-the-start scenario ---")
state0_bad = np.array([0.0, 0.0, 5.0, 0.0, 0.0, -60.0, 25000.0])
_, _, controls_bad, _, failed_bad, = simulate_mpc(
    state0_bad, params, weights, target, N, dt, max_steps=3
)
print("solve_failed:", failed_bad)
print("first control applied (should be hover-like fallback)::", controls_bad[0])
expected_hover_thrust = params.initial_mass * params.g
print("expected hover thrust:", expected_hover_thrust)
print("simulation kept running (no crash), real steps taken:", len(controls_bad))
