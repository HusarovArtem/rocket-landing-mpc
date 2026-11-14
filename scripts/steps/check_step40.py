from experiments.runners import run_pid, nominal_scenario, run_mpc
from rocket_landing.controllers.mpc import MpcWeights
from rocket_landing.params import VehicleParams

params = VehicleParams()
weights = MpcWeights()
state0 = nominal_scenario()

_, _, summary_pid = run_pid(state0, params)
_, _, summary_mpc = run_mpc(state0, params, weights)


print("PID summary:", summary_pid)
print("MPC summary:", summary_mpc)