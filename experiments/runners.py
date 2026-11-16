import numpy as np

from rocket_landing.controllers.mpc import simulate_mpc
from rocket_landing.controllers.pid import PIDController
from rocket_landing.metrics import landing_summary
from rocket_landing.params import VehicleParams
from rocket_landing.reference import LandingTarget
from rocket_landing.simulation import simulate_pid

def nominal_scenario():
    """
    The ONE standard starting state used across steps 27-39 for BOTH
    PID and MPC: x=100m, y=50m, z=500m up, falling at vz=-50m/s,
    full tank(initial_mass).

    "Standardizing scenarios" (step 40) means exactly this: instead of
    every check/plot script re-typing same 7 numbers by hand (and
    risking a typo that quietly makes a comparison unfair. Mode
    scenarios (off-nominal starts, wind, etc.) get added here the
    same way in later experiment steps (42+).
    """
    params = VehicleParams()
    return np.array([100.0, 50.0, 500.0, 0.0, 0.0, -50.0, params.initial_mass])


def run_pid(state0, params, dt=0.05, wind=None):
    """
    Runs the PID baseline (same gains as steps 25-27/29) on `state0` and
    returns (times, states, summary) - summary is the SAME landing_summary()
    dict used everywhere since step 28, plus a "controller" label so results
    from run_pid() and run_mpc() can be told apart once mixed together table (step 42)
    """
    pid_vz = PIDController(kp=1500.0, ki=50.0, kd=800.0)
    pid_x = PIDController(kp=500.0, ki=0.0, kd=2000.0)
    pid_y = PIDController(kp=500.0, ki=0.0, kd=2000.0)

    times, states = simulate_pid(state0, pid_vz, pid_x, pid_y, params, dt, wind)

    target = LandingTarget()
    summary = landing_summary(times, states, target, state0[6])
    summary["controller"] = "PID"

    return times, states, summary


def run_mpc(state0, params, weights, N=30, dt=0.1, max_steps=2000, warm_start=True, wind=None):
    """
    Runs the full receding-horizon MPC loop (steps 36-38) on `state0`
    and returns (times, states, summary) - the SAME landing_summary()
    dict as run_pid(), so the two are directly comparable side by side,
    plus MPC specific extras (solver_failures, avg_iterations) that don't
    apply to PID at all.
    """
    target = LandingTarget()

    times, states, controls, iter_counts, solve_failed = simulate_mpc(
        state0, params, weights, target, N, dt, max_steps=max_steps, warm_start=warm_start, wind=wind

    )

    summary = landing_summary(times, states, target, state0[6])
    summary["controller"] = "MPC"
    summary["solver_failures"] = sum(solve_failed)
    summary["avg_iterations"] = sum(iter_counts) / len(iter_counts)

    return times, states, summary
