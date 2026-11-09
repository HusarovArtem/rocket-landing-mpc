from casadi import casadi

from CODE_TO_BE_REWRITTEN import x_next_sym, hover_thrust, control0, numpy_result0
from rocket_landing.controllers.mpc import casadi_rk4_step
from rocket_landing.integrators import rk4_step
from rocket_landing.params import VehicleParams
import casadi as ca
import numpy as np

p = VehicleParams()

# Build CasADi Function wrapping casadi_rk4_step, same pattern as the
# Function we built for casadi_dynamics in step 30.
x_sym = ca.SX.sym("x", 7)
u_sym = ca.SX.sym("u", 3)
dt_sym = ca.SX.sym("dt")
x_next_sym = casadi_rk4_step(x_sym, u_sym, p, dt_sym)
f_step = ca.Function("f_step", [x_sym, u_sym, dt_sym], [x_next_sym])

# Test 1: hover, one step. Thrust exactly cancels gravity, no tilt.
# Expect the position/velocity to barely move (hover ~ holds still,
# aside from tiny RK4 differences), and mass to drop by the expected
# mass-flow amount over dt.
state0 = np.array([0.0,0.0,500.0,0.0,0.0,0.0,p.initial_mass])
hover_thrust = p.initial_mass * p.g
control0 = np.array([hover_thrust, 0.0, 0.0])
dt = 0.1


casadi_result0 = np.array(f_step(state0, control0, dt)).flatten()
numpy_result0 = rk4_step(state0, control0, p, dt)

print("casadi (hover):", casadi_result0)
print("numpy (hover):", numpy_result0)

# Test 2: tilted thrust, not a round number, and a bigger dt - makes
# sure agreement isn't a fluke of the simple hover case or a tiny dt.
state1 = np.array([10.0, -5.0, 300.0, 2.0, -1.0, -20.0,24000.0])
control1 = np.array([25000.0, 0.1, -0.05])
dt1 = 0.5

casadi_result1 = np.array(f_step(state1, control1, dt1)).flatten()
numpy_result1 = rk4_step(state1, control1, p, dt1)

print("casadi (hover):", casadi_result1)
print("numpy (hover):", numpy_result1)
