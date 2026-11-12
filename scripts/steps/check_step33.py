import numpy as np
import casadi as ca

from rocket_landing.controllers.mpc import build_mpc_nlp, casadi_rk4_step
from rocket_landing.params import VehicleParams

N = 3
dt = 0.1
params = VehicleParams()

opti, X, U = build_mpc_nlp(params, N, dt)

# Hover-like test point: thrust exactly balances gravity at the
# INITIAL mass (same numbers as the step-30 hover check), straight up.
x0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 25000.0])
u0 = np.array([params.initial_mass * params.g, 0.0, 0.0])

opti.subject_to(X[:, 0] == x0)
for k in range(N):
    opti.subject_to(U[:, k] == u0)

# Initial guess for IPOPT: without this, every entry defaults to 0,
# including mass - and the dynamics divide thrust by mass, so IPOPT
# would hit a division by zero before even starting to iterate.
opti.set_initial(X, np.tile(x0.reshape(-1, 1), (1, N + 1)))
opti.set_initial(U, np.tile(u0.reshape(-1, 1), (1, N)))

opti.minimize(0)  # no real objective yet (that's step 35) - the
                   # problem is already fully pinned down by the
                   # equality constraints above, so any objective
                   # (even 0) just lets IPOPT confirm feasibility.
opti.solver("ipopt")
sol = opti.solve()

X_nlp = sol.value(X)

# Manual reference: chain casadi_rk4_step by hand with the SAME
# inputs, outside of any Opti problem.
x = x0
X_manual = [x0]
for k in range(N):
    x_expr = casadi_rk4_step(x, u0, params, dt)
    x = np.array(ca.DM(x_expr)).flatten()
    X_manual.append(x)
X_manual = np.array(X_manual).T

print("X from NLP:\n", X_nlp)
print("X from manual chain:\n", X_manual)
print("max abs difference:", np.max(np.abs(X_nlp - X_manual)))
