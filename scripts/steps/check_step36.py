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

times, states, controls = simulate_mpc(state0, params, weights, target, N, dt, max_steps=5)

print("Real steps taken:", len(controls))
print("times:", times)
print("\nReal states (x, y, z, vz, mass):")
for i, s in enumerate(states):
    print(f"    i={i}: x={s[0]:7.3f} y={s[1]:7.3f} vz={s[5]:7.3f} mass={s[6]:9.3f}")

print("\nControls ACTUALLY applied (thrust, alpha, beta):")
for i, c in enumerate(controls):
    print(f"    i={i}: thrust={c[0]:10.3f} alpha={c[1]:8.5} beta={c[2]:8.5}")


control_changes = np.diff(controls, axis=0)
print("\n Control vector changed between consecutive real steps (not frozen):",
      np.any(np.abs(control_changes) > 1e-6))
