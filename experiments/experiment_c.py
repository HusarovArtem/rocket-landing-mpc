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

def experiment_c_model_mismatch():
    """
    The REAL engine is weaker and less effictient than the
    MPC'c internal model assumes: 10% less max thrust
    (360 kN instead of 400 kN) and 10% rowse Isp (270s instead of 300s),
    i.e. burns more fuel for the same thrust than the model predicts)
    This models a controller flying with an outdated/approximate concern.


    PID is run on REAL (weaker) engine directly - PID has no internal
    engine model to be WRING about, it just reacts to the true physics as
    given, so this is really "PID o a weaker rocket", not "PID fooled by a bad model"
    (there's no PID equivalent of plant-model mismatch here).
    """
    model_params = VehicleParams() # what the MPC's NLP believes
    real_params = VehicleParams()  # what the engine actuallu does
    real_params.max_thrust = 0.9 * model_params.max_thrust
    real_params.isp = 0.9 * model_params.isp

    weights = MpcWeights()
    state0 = nominal_scenario()

    _, _, pid_real = run_pid(state0, real_params)
    _, _, mpc_mismatch = run_mpc(state0, model_params, weights, real_params=real_params)

    print(f"{'Metric':<28}{'PID (real engine)':>20}{'MPC (model mismatch)':>22}")

    for key, label in (("position_error_m", "Position error [m]"),
                       ("touchdown_speed_mps", "Touchdown speed [m/s"),
                       ("fuel_used_kg", "Fuel used [kg]")):
        print(f"{label:<28}{pid_real[key]:>20.2f}{mpc_mismatch[key]:>22.2f}")

    return pid_real, mpc_mismatch