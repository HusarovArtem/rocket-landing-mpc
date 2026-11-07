import casadi as ca
import numpy as np

from rocket_landing.params import VehicleParams

def casadi_dynamics(state, control, params):
    """
    Symbolic version of dynamics() from dynamics.py. Returns the
    time-derivative of the state (d(state)/dt), length 7, same order as
    state itself - EXACTLY the same physical model (gravity + thrust +
    fuel burn, no air resistance), just written with CasADI's own
    functions so it can be plugged into a symbolic optimization problem later
    """
    vx = state[3]
    vy = state[4]
    vz = state[5]
    mass = state[6]

    thrust = control[0]
    alpha = control[1]
    beta = control[2]

    # --- gravity: identical idea to dynamics.py - straight down, doesn't
    # depend on mass or control at all.
    ax_gravity = 0.0
    ay_gravity = 0.0
    az_gravity = 0.0 - params.g

    # --- thrust direction: same formula as thrust_direction() in
    # dynamics.py (d/alpha/beta) = [sin(a)cos(b), sin(b), cos(a)cos(b)]),
    # but using ca.sin/ca.cos (CasADi's trig) instead of np.sin/np.cos.
    dir_x = ca.sin(alpha) * ca.cos(beta)
    dir_y = ca.sin(beta)
    dir_z = ca.cos(alpha) * ca.cos(beta)


    # -- thrust acceleration: force / mass, same Newton's-2nd-law idea
    # as dynamics.py ( a = F/m, done per-component).
    ax_thrust = thrust * dir_x / mass
    ay_thrust = thrust * dir_y / mass
    az_thrust = thrust * dir_z / mass

    ax = ax_gravity + ax_thrust
    ay = ay_gravity + ay_thrust
    az = az_gravity + az_thrust

    # --- mass flow: identical formula to dynamics.py.
    mass_rate = -thrust / (params.isp * params.g0)

    # ca.vertcat is CasADi's version of np.concatenate: it stacks scalar
    # expressions into ONE column vector, in the order given here
    return ca.vertcat(vx, vy,vz, ax, ay, az, mass_rate)