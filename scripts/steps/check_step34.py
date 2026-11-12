import numpy as np

from rocket_landing.controllers.mpc import build_mpc_nlp, add_state_and_actuator_constraints
from rocket_landing.params import VehicleParams

params = VehicleParams()
x0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 25000.0])

# --- Test 1: thrust gets pinned to max_thrust, not beyond --------
opti, X, U = build_mpc_nlp(params, N=1, dt=0.1)
add_state_and_actuator_constraints(opti, X, U, params)
opti.subject_to(X[:, 0] == x0)
opti.minimize(-U[0, 0]) # "maximize thrust" == minimize its negative
opti.set_initial(X, np.tile(x0.reshape(-1, 1), (1, 2)))
opti.set_initial(U, np.array([[params.max_thrust], [0.0], [0.0]]))
opti.solver("ipopt")
sol = opti.solve()
print("Test 1 - thrust found:", sol.value(U[0, 0]), "| limit (max_thrust):", params.max_thrust)



# --- Test 2: alpha gets pinned to max_tilt_angle, not beyond --------
opti2, X2, U2 = build_mpc_nlp(params, N=1, dt=0.1)
add_state_and_actuator_constraints(opti2, X2, U2, params)
opti2.subject_to(X2[:, 0] == x0)
opti2.minimize(-U2[1, 0]) # "maximize alpha"
opti2.set_initial(X2, np.tile(x0.reshape(-1, 1), (1, 2)))
opti2.set_initial(U2, np.array([[params.min_thrust], [0.0], [0.0]]))
opti2.solver("ipopt")
sol2 = opti2.solve()
print("Test 2 - alpha found:", sol2.value(U2[1, 0]), "| limit (max_tilt_ange):", params.max_tilt_angle)



# --- Test 3: height >= 0 blocks and impossible through-the-ground step --------
opti3, X3, U3 = build_mpc_nlp(params, N=1, dt=0.5)
add_state_and_actuator_constraints(opti3, X3, U3, params)
x0_ground = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -50.0, 25000.0])
u0_weak = np.array([params.min_thrust, 0.0, 0.0])
opti3.subject_to(X3[:, 0] == x0_ground)
opti3.subject_to(U3[:, 0] == u0_weak)
opti3.minimize(0)
opti3.minimize(0)
opti3.set_initial(X3, np.tile(x0_ground.reshape(-1, 1), (1,2)))
opti3.set_initial(U3, u0_weak.reshape(-1,1))
opti3.solver("ipopt")

try:
    opti3.solve()
    print("Test 3 - UNEXEPECTED: solver found a feasible point (height >= 0 did NOT BLOCK it")
except RuntimeError:
    print("Test 3 - solver correctly reports INFEASIBLE (height >= 0 is blockin a through the ground prediction), as expected")


# --- Test 4: mass >= dry_mass blocks burning past an empty tank ------
opti4, X4, U4 = build_mpc_nlp(params, N=1, dt=1.0)
add_state_and_actuator_constraints(opti4, X4, U4, params)
x0_low_fuel = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, params.dry_mass + 1.0])
u0_full = np.array([params.max_thrust, 0.0, 0.0])
opti4.subject_to(X4[:, 0] == x0_low_fuel)
opti4.subject_to(U4[:, 0] == u0_full)
opti4.minimize(0)
opti4.set_initial(X4, np.tile(x0_low_fuel.reshape(-1,1),(1,2)))
opti4.set_initial(U4, u0_full.reshape(-1,1))
opti4.solver("ipopt")

try:
    opti4.solve()
    print("Test 4 - UNEXEPCTEDL solver found a feasible point (mass >= drymass DID NOT block it")
except RuntimeError:
    print("Test 4 - solver correctly reports INFEASIBLE (mass >= drymass is blockin over-burning fuel), as expected")

