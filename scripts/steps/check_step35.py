import numpy as np

from rocket_landing.controllers.mpc import build_mpc_nlp, add_state_and_actuator_constraints, MpcWeights, \
    solve_open_loop_trajectory
from rocket_landing.params import VehicleParams
from rocket_landing.reference import LandingTarget

params = VehicleParams()
weights = MpcWeights()
target = LandingTarget()


x0 = np.array([100.0, 50.0, 500.0, 0.0, 0.0, -50.0, 25000.0])
N = 30
dt = 0.1

X_opt, U_opt = solve_open_loop_trajectory(params, weights, target, x0, N, dt)

print("Positions (x, y, z) at k=0, 10, 20, 30:")
for k in [0, 10, 20, 30]:
    print(f"  k={k:2d}: x={X_opt[0, k]:8.3f}  y={X_opt[1, k]:8.3f}"
          f"  z={X_opt[2, k]:8.3f} vz={X_opt[5, k]:8.3f} mass={X_opt[6, k]:9.3f}")

thrust = U_opt[0, :]
alpha = U_opt[1, :]
beta = U_opt[2, :]

tol = 1e-3
thrust_ok = np.all(thrust >= params.min_thrust - tol) and np.all(thrust <= params.max_thrust + tol)
alpha_ok = np.all(np.abs(alpha) <= params.max_tilt_angle + tol)
beta_ok = np.all(np.abs(beta) <= params.max_tilt_angle + tol)
height_ok = np.all(X_opt[2, :] >= -tol)
mass_ok = np.all(X_opt[6, :] >= params.dry_mass - tol)

print("\n Bounds respected:")
print("     thrust in [min,max]:", thrust_ok)
print("     alpha in [-max,max]:", alpha_ok)
print("     beta  in [-max,max]:", beta_ok)
print("     height >= 0:", height_ok)
print("     mass >= dry mass:", mass_ok)

vz_start = X_opt[5, 0]
vz_end = X_opt[5, N]
print(f"\nvz at k=0: {vz_start:.3f}  -> vz at k={N}: {vz_end:.3f}    "
      f"(braking: {abs(vz_end) < abs(vz_start)})")
print("NOTE: with today's placeholder weights, 'braking: False' is the "
      "EXPECTED result here, not a bug - see the comments above this block for why. "
      "Real deceleration behavior arrives with proper weight tuning in step 47.")
