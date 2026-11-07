import casadi as ca
import numpy as np

from CODE_TO_BE_REWRITTEN import xdot_sym, hover_thrust
from rocket_landing.params import VehicleParams
from rocket_landing.dynamics import dynamics
from rocket_landing.controllers.mpc import casadi_dynamics

p = VehicleParams()

# Build symbolic placeholders, then a CasADi "Function" that can
# be called later with ACTUAL numbers, like a normal Python
# function - this is the standard CasADi pattern we'll reuse in
# the steps 31-39
x_sym = ca.SX.sym("x", 7)
u_sym = ca.SX.sym("u", 3)
xdot_sym = casadi_dynamics(x_sym, u_sym, p)
f = ca.Function("f", [x_sym, u_sym], [xdot_sym])

# Test 1: hover - thrust exactly cancels gravity, no tilt. Expect
# acceleration ~0 in all 3 axes (just like the hover test from
# step 12), and the SAME mass_rate formula as dynamics.py.
state0 = np.array([0.0, 0.0, 500.0, 0.0, 0.0, 0.0, p.initial_mass])
hover_thrust = p.initial_mass * p.g
control0 = np.array([hover_thrust, 0.0, 0.0])

print("casadi (hover):", np.array(f(state0, control0)).flatten())
print("numpy (hover):", dynamics(state0, control0, p))

# Test 2 tilted thrust, not a round number - makes sure this isn't
# an accident that only works for the simple hover case.
state1 = np.array([10.0, -5.0, 300.0, 2.0, -1.0, -20.0, 24000.0])
control1 = np.array([25000.0, 0.1, -0.005])

print("casadi (tilted):", np.array(f(state1, control1)).flatten())
print("numpy (tilted):", dynamics(state1, control1, p))