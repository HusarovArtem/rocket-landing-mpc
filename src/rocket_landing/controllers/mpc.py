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

def casadi_rk4_step(state, control, params, dt):
    """
    Symbolic version of rk4_step() from integrators.py. Advances `state`
    forward by one step `dt`, using the SAME classic RK4 formula (4
    derivative samples, combined with wights 1/2/2/1 over 6) - but
    written with CasADi expression (casadi_dynamics instead of
    dynamics) so it can live inside a symbolic optimization problem.
    """
    # k1: derivative at the start of the step - identical idea to
    # rk4_step(), just calling casadi_dynamics instead of dynamics
    k1 = casadi_dynamics(state, control, params)

    # k2: derivative at a trial half-step forward using k1
    k2 = casadi_dynamics(state + dt / 2 * k1, control, params)

    # k3: derivative at another trial half-step, this time using k2
    k3 = casadi_dynamics(state + dt / 2 * k2, control, params)

     # k4: derivative at a trialfull step forward using k3
    k4 = casadi_dynamics(state + dt * k3, control, params)

    # combine the four slope estimates with the classic RK4 weights
    return state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

