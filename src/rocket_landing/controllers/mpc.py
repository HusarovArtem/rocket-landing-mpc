import casadi as ca
import numpy as np

from rocket_landing.actuators import clamp_control
from rocket_landing.integrators import rk4_step


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

def build_mpc_nlp(params, N, dt):
    """
    Builds the SKELETON of the MPC optimization problem (NLP):
    the decision variables (predicted state and control trajectories over
    the horizon) and the dynamics constraints that tie them together.

    Nothing else yet - no cost function, no actuator/state bounds, no
    initial-condition constraint. Those come in later steps (34: bounds,
    35: cost + initital condition + solve). THis step only answers:
    "what are we optimizing OVER, and what makes a candidate trajectory
    physically possible in the first place?"

    N - number of control steps in the horizon (prediction horizon
    length, e.g. 30)
    dt - duration of one step in seconds (same model used inside
    casadi_rk4_step

    Returns (opti, X, U) so later steps can add bounds,
    the intial-state constaint, and the cost function on top f the SAME
    problem - opti.subject_to()/opti.minimie() can be called again from
    outside the function.
    """
    opti = ca.Opti()

    # State trajectory: one state column (length 7) per horizon point -
    # N+1 pints total, k=0 ("now") through k=N (end of horizon).
    X = opti.variable(7, N + 1)

    # Control trajectory: one state column (length 3) per horizon
    # STEP N columns total , k = 0 through k=N-1. There is no control
    # decision "after" the last state, same reasoning, as terminal_cost
    U = opti.variable(3, N)

    # Dynamics constraint: force every consecutive pair of steps to be
    # consistent with the discretized physics (casadi_rk4_step) under
    # the chosen control at that step. This is what turns "N+1 arbitrary
    # state columns into "one physically-possible predicted trajectory".
    for k in range(N):
        x_next_predicted = casadi_rk4_step(X[:, k], U[:, k], params, dt)
        opti.subject_to(X[:, k+1] == x_next_predicted)

    return opti, X, U


def add_state_and_actuator_constraints(opti, X, U, params):
    """
    Adds the SAME physical limits already used by clamp_control() for the
    simulator - but here they go directly INTO the optimizer as hard
    constraints it must respect WHILE searching for a solution, instead of
    clipping an already chosen control AFTER the fact.
    """
    thrust = U[0, :]
    alpha = U[1, :]
    beta = U[2, :]

    opti.subject_to(opti.bounded(params.min_thrust, thrust, params.max_thrust))
    opti.subject_to(opti.bounded(-params.max_tilt_angle, alpha, params.max_tilt_angle))
    opti.subject_to(opti.bounded(-params.max_tilt_angle, beta, params.max_tilt_angle))

    height = X[2, :]
    mass = X[6, :]

    opti.subject_to(height >= 0)
    opti.subject_to(mass >= params.dry_mass)


def solve_open_loop_trajectory(params, weights, target, x0, N, dt, u_prev=None, X_guess=None, U_guess=None):
    """
    Builds and solves ONE full MPC optimization problem, from a single
    starting state x0, over a fixed horizon of N steps - NOT the
    receding-horizon MPC loop yet (that's step 36). This is just a
    "preview": what trajectory would the optimizer plan RIGHT NOW,
    before any close-loop correction ever happens ?

    This wires together every piece from steps 32-34:
        - build_mpc_nlp(): the decision variables + dynamics constraints
        - add_state_and_actuator_constraints(): the physical bounds
        - stage_cost() / terminal_cost(): what "good" means
    plus the ONE thing build_mpc_nlp() deliberately left out: pinning
    the very first state to a concrete starting point.

    Returns (X_opt, U_opt) as plain numpy arrays (7 x (N+1) and 3 x N)
    so the result can be printed/plotted directly.

    *** Adding new optionall argument: u_prev as for step 36

    u_prev is the control that was Actuallt applied to the rocket just before
    this solve (needed so the delta_u penalty at k=0 compares against real, once
    this gets called repeatedly inside a closed loop - step 36). If u_prex is None
    (a one-off, standalone comparing its control against itself, same as before
    giving a harmless zero, detat_u at k=0


    *** Adding two new optional params X_guess and U_guess  (step 37)

    This is what makes warm-starting possible: a solve from on real step
    can hand its OWN solution forward as the starting guess for the NEXT real
    steps. Since consecutive solves along a receding horizon are very similar
    to each other (same physics, nearly the same starting state), this gives
    IPOPT a near-connect answer to start from, instead of guessing from scratch every single time.
    """

    opti, X, U = build_mpc_nlp(params, N, dt)
    add_state_and_actuator_constraints(opti, X, U, params)

    # The initial-state constraint: "The predicted trajectory must start
    # from where the rocket ACTUALLY is right now" - without this, X[:, 0]
    # would be free to be anything the optimizer likes
    opti.subject_to(X[:,0] == x0)

    initial_mass = x0[6]

    cost = 0
    for k in range(N):
        if k > 0:
            control_prev = U[:, k-1]
        elif u_prev is not None:
            control_prev = u_prev
        else:
            control_prev = U[:, 0]
        cost = cost + stage_cost(X[:, k], U[:, k], control_prev, target, weights)
    cost = cost + terminal_cost(X[:, N], initial_mass, target, weights)

    opti.minimize(cost)

    if X_guess is None:
        X_guess = np.tile(np.array(x0).reshape(-1, 1), (1, N+1))
    if U_guess is None:
        hover_u = np.array([params.initial_mass * params.g, 0.0,0.0])
        U_guess = np.tile(hover_u.reshape(-1,1), (1,N))

    opti.set_initial(X, X_guess)
    opti.set_initial(U, U_guess)

    opti.solver("ipopt", {"print_time": False}, {"print_level": 0})
    sol = opti.solve()

    return sol.value(X), sol.value(U), sol.stats()["iter_count"]


def simulate_mpc(state0, params, weights, target, N, dt, max_steps=1000, warm_start= True):
    """
    The receding-horizon MPC loop (step 36): at EVERY real time step,
    solve a FRESH NLP from the rocket's current REAL state, apply ONLY
    the first control U[:, 0] from that plan to the real rocket, advance
    one real step, then THROW AWAY the rest of the plan and repeat from
    the new state. This is the mechanizm discussed as "receding
    horizon"- the planning window always starts at "now and slides
    forward with time, instead of being computed once and blindly  followed

    The REAL rocket is advanced with rk4_step() (integrators.py) and
    clamp_control() (actuators.py) - the SAME, independent, non-CasADi
    functions already ised by simulate_pid() - deliberately NOT
    casadi_rk4_step(). This keeps "what the MPC predicts" and "wha
    actually happends to the rocket" as separate implementations
    (same reasoning as step 30 - needed later for the model-mismatch
    experiment, step 45). clamp_control() is a cheap safety net here:
    the NLP's own bounds should already guarantee U[:, 0] respects the
    limits, clamping just protects against solver tolerance slack.

    When warm_start = True (the new default - step 37), each real step's
    solve hans its OWN solution forward - SHIFTED by one step - as the starting
    guess for the NEXT real step's above.

    Returns:
        times       - 1D array, one timestamp per REAL step (like simulate_pid)
        states      - 2D array, one row per REAL state
        controls    - 2D array, one row per control ACTUALLY applied
                    (only ever U[:, 0] of each solve - never the rest o the plan)
    """
    times = [0.0]
    states = [np.array(state0, dtype=float)]
    controls = []
    iter_counts=[]

    u_prev = None # no REAL prev control exists yet at the first step
    X_guess = None
    U_guess = None

    for i in range(max_steps):
        current_state = states[-1] # last element (current)

        X_opt, U_opt, iters = solve_open_loop_trajectory(
            params, weights, target, current_state, N, dt,
            u_prev=u_prev, X_guess=X_guess, U_guess=U_guess
        )
        iter_counts.append(iters)
        control = clamp_control(U_opt[:, 0], params)

        next_state = rk4_step(current_state, control, params, dt)
        times.append(times[-1] + dt)
        states.append(next_state)
        controls.append(control)

        u_prev = control

        if warm_start:
            X_guess = np.hstack([X_opt[:, 1:], X_opt[:, -1:]])
            U_guess = np.hstack([U_opt[:, 1:], U_opt[:, -1:]])
        else:
            X_guess = None
            U_guess = None

        if next_state[2] <= 0.0: # z <= 0.0
            break
        if next_state[6] <= params.dry_mass: # mass <= dry mass
            break

    return np.array(times), np.array(states), np.array(controls), iter_counts