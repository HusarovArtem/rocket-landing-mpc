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

class MpcWeights:
    """
    Tuning knobs for the MPC cost function - how much each kind of
    "badness" matters relative to the others. Every weight here is a
    single scalar applied the SAME to all 3 axes (x, y, z) - a real MPC
    could use a full matrix (different weight per axis), but a single
    scalar per term is the simplest version that still lets us express
    the full cost formula from the thesis plan. Can be replaced with a
    proper per-axis/matrix version later if needed - not today.

    These starting numbers are placeholders, not tuned yet (tuning is
    step 47, "MPC parameter sweep") - they only need to be "reasonable
    enough" for us to manually check the cost formula is wired up
    correctly
    """


    def __init__(self):
        self.Qr = 1.0    # running cost: position error (per step k)
        self.Qv = 1.0    # running cost: velocity error (per step K)
        self.R = 1e-6    # running cost: control effort size (thrust is
                         # a bug number in Newtons, so this weight must
                         # be tiny or it would completely dominate)
        self.S = 1e-6    # running cost: control CHANGE (delta_u) between
                         # consecutive steps - penalizes jerky control

        self.S = 1e-6    # running cost: control CHANGE (delta_u) between
                         # consecutive steps - penalizes jerky control
        self.Qf = 10.0   # terminal cost: final velocity (bigger than Qv,
                         # same reasoning - a "soft landing" needs small
                         # final velocity specifically, not just on average
        self.w_m = 0.001 # terminal cost: fuel used (initial_mass - final
                         # mass) - small weight, fuel saving matters but
                         # much less than actually landing well

def stage_cost(state, control, control_prev, target, weights):
    """
    THe cost added at ONE time step k along the MPN horizon (the "running"
    or "stage" cost - this gets summed over every step in the horizon

        cost_k = Qr*||position_error||^2 + Qv*||velocity_error||^2
                 + R*||control||^2 + S*||control-control_prev||^2

    Each term penalizes a DIFFERENT kind of "badness": being far from the
    target (position/velocity error), and using large or jerky control
    (control size / control change). ca.sumsqr(v) is CasADi's shortcut
    for sum of v[1]**2 over all entires" - i.e. ||v||^2, the squared
    Euclidean length of a vector
    """
    position = state[0:3]
    velocity = state[3:6]

    position_error = position - target.position
    velocity_error = velocity - target.velocity

    position_cost = weights.Qr * ca.sumsqr(position_error)
    velocity_cost = weights.Qv * ca.sumsqr(velocity_error)
    control_cost = weights.R * ca.sumsqr(control)

    delta_u = control - control_prev
    delta_control_cost = weights.S * ca.sumsqr(delta_u)

    return position_cost + velocity_cost + control_cost + delta_control_cost


def terminal_cost(state, initial_mass, target, weights):
    """
    The cost added ONCE, only at the very END of the horizon (step N) -
    this is what actually rewards the landing target, not just
    passing near it along the way (that's what stage_cost already covers
    every step):

        cost_N = Qf*||position_error_N||^2 + Pf*||velocity_N||^2
                 + w_m*(initial_mass - mass_N)

    There's no "control term here - by definition there is no control
    decision happening after the last state of the horizon, so nothing
    like control/delta_control_cost applies to this point.
    """
    position = state[0:3]
    velocity = state[3:6]
    mass = state[6]

    position_error = position - target.position

    position_cost = weights.Qf * ca.sumsqr(position_error)
    velocity_cost = weights.Qf * ca.sumsqr(velocity)
    fuel_cost = weights.w_m * (initial_mass - mass)

    return position_cost + velocity_cost + fuel_cost
