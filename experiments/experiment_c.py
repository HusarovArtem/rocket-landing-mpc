import numpy as np

from experiments.runners import nominal_scenario, run_pid, run_mpc
from rocket_landing.controllers.environment import Wind
from rocket_landing.controllers.mpc import simulate_mpc, MpcWeights
from rocket_landing.params import VehicleParams

def scenario_with_initial_error(vz_error=0.0, mass_error=0.0):
    """
    Same standardized scenario as nominal_scenario() (step 40), but with
    an added ERROR in the initial velocity and/or mass - simulating
    imperfect sensor/estimation of the rocket's OWN starting condition
    (e.g. the navigation system's velocity estimate was off, or less
    fuel was actually onboard than the flight plan assumed.)
    """
    state0 = nominal_scenario().copy()
    state0[5] += vz_error
    state0[6] +=mass_error
    return state0


def experiment_c_initial_error():
    """
    Experiment C variant: robustness to an ERROR in the
    initial velocity/mass estimate. Same standardized scenario as
    Experiment A, but the rocket actually starts falling
    10 m/s faster (vz=-60) instead of -50) and with 500 kg less fuel
    than the nominal case assumed.
    """
    params = VehicleParams()
    weights = MpcWeights()
    state0 = nominal_scenario()
    state0_error = scenario_with_initial_error(vz_error=-10.0, mass_error=-500.0)

    _, _, pid_nominal = run_pid(state0, params)
    _, _, pid_error = run_pid(state0_error, params)
    _, _, mpc_nominal = run_mpc(state0, params, weights)
    _, _, mpc_error = run_mpc(state0_error, params, weights)

    print(f"{'Metric':<28}{'PID (nominal)':>16}{'PID (error)':>16}"
          f"{'MPC (nominal)':>16}{'MPC (error)':>14}")

    for key, label in (("position_error_m", "Position error [m]"),
                       ("touchdown_speed_mps", "Touchdown speed [m/s"),
                       ("fuel_used_kg", "Fuel used [kg]")):
        print(f"{label:<28}{pid_nominal[key]:>16.2f}{pid_error[key]:>16.2f}"
              f"{mpc_nominal[key]:>16.2f}{mpc_error[key]:>16.2f}")

    return pid_nominal, pid_error, mpc_nominal, mpc_error