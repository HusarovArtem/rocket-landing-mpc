import numpy as np

from experiments.runners import nominal_scenario, run_pid, run_mpc
from rocket_landing.controllers.environment import Wind
from rocket_landing.controllers.mpc import simulate_mpc, MpcWeights
from rocket_landing.params import VehicleParams


def experiment_a():
    """
    Experiment A: nominal comparison of PID vs MPC on the
    SAME standardized starting scenario (step 40) - same vehicle
    parameters, same starting state, same landing target, no
    disturbances yet (wind/model-mismatch are later experiments
    This is the first real use of run_pid()/run_mpc() for their
    intended purpose: a side-by-side comparison table instead of two
    separately-printed dicts.

    Prints a small table and returns (summary_pid, summary_mpc_ so a
    later step (or the thesis chapter 8.1 test) can reuse the numbers
    without re-running the simulation.
    """
    params = VehicleParams()
    weights = MpcWeights()
    state0 = nominal_scenario()

    _, _, summary_pid = run_pid(state0, params)
    _, _, summary_mpc = run_mpc(state0, params, weights)

    print(f"{'Metric':<28}{'PID':>14}{'MPC':>14}")
    print("-" * 56)
    print(f"{'Flight time [s]':<28}{summary_pid['flight_time_s']:>14.2f}"
          f"{summary_mpc['flight_time_s']:>14.2f}")
    print(f"{'Position error [m]':<28}{summary_pid['position_error_m']:>14.2f}"
          f"{summary_mpc['position_error_m']:>14.2f}")
    print(f"{'Touchdown speed [m/s]':<28}{summary_pid['touchdown_speed_mps']:>14.2f}"
          f"{summary_mpc['touchdown_speed_mps']:>14.2f}")
    print(f"{'Fuel used [kg]':<28}{summary_pid['fuel_used_kg']:>14.2f}"
          f"{summary_mpc['fuel_used_kg']:>14.2f}")
    print(f"{'Solver failures':<28}{'n/a':>14}{summary_mpc['solver_failures']:>14}")
    print(f"{'Avg. IPOPT iterations':<28}{'n/a':>14}{summary_mpc['avg_iterations']:>14.1f}")

    return summary_pid, summary_mpc

def experiment_a_wind():
    """
    Repeats Experiment A (step 42) with a constant horizontal crosswind
    disturbance (step 43) - same standardized scenario, same two
    controllers, but now the REAL simulator experiences a sideways
    acceleration disturbance that NEITHER controller's internal model
    knows about in advance.
    """
    params = VehicleParams()
    weights = MpcWeights()
    state0 = nominal_scenario()
    wind = Wind(ax=0.3, ay=0.0)

    _, _, pid_no_wind = run_pid(state0, params)
    _, _, pid_wind = run_pid(state0, params, wind=wind)
    _, _, mpc_no_wind = run_mpc(state0, params, weights)
    _, _, mpc_wind = run_mpc(state0, params, weights, wind=wind)

    print(f"{'Metric':<28}{'PID (no wind)':>16}{'PID (wind)':>16}"
          f"{'MPC (no wind)':>16}{'MPC (wind)':>16}")
    for key, label in (("position_error_m", "Position error [m]"),
                       ("touchdown_speed_mps", "Touchdown speed [m/s"),
                       ("fuel_used_kg", "Fuel used [kg]")):
        print(f"{label:<28}{pid_no_wind[key]:>16.2f}{pid_wind[key]:>16.2f}"
              f"{mpc_no_wind[key]:>16.2f}{mpc_wind[key]:>16.2f}")

    return pid_no_wind, pid_wind, mpc_no_wind, mpc_wind