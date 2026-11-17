import math


from experiments.runners import nominal_scenario, run_pid, run_mpc
from rocket_landing.controllers.mpc import simulate_mpc, MpcWeights
from rocket_landing.params import VehicleParams


def _run_trial(label, state0, Qv=1.0, Qf=10.0, S=1e-6, N=30,dt=0.1, tilt_deg=20.0):
    """
    One MPC trial on nominal_scenario() with ONE parameter changed from
    the step-32 placeholder defaults (Qr=1.0, Qv=1.0, R=1e-6, S=1e-6.
    Qf=10.0, N=30, dt=0.1, max_tilt_angle=20deg) - everything else stays
    default, so any difference in the result is attributable to that one
    change (the "vary one at a time" approach from the thesis plan,
    section 6, Experiment D).
    """
    params = VehicleParams()
    params.max_tilt_angle = math.radians(tilt_deg)
    weights = MpcWeights()
    weights.Qv, weights.Qf, weights.S = Qv, Qf, S

    _, _, summary = run_mpc(state0, params, weights, N=N, dt=dt)
    summary["label"] = label
    return summary

def experiment_d_sweep():
    """
    Experiment D (step 47): vary horizon length (N), terminal weight (Qf),
    control-change penalty (S), control-recompute period (dt), and
    thrust-tilt limit one at a time around the step-32 placeholder
    defaults - plus Qv (running velocity-error weight), which is NOT one
    of the thesis plan's 5 listed knobs but turned out, while prototyping
    this step, to be the dominant lever (see note below) and therefore
    deserves its own plave in this sweep.

    Prints ALL results, but FLAGS any row where landed=FALSE as invalid
    for comparison purposes (see the metrics.py fix above) - a crashed/
    unfinished run is not "better" just because its reported touchdown
    speed happens to look small.
    """
    state0 = nominal_scenario()

    trials = [
        _run_trial("baseline (step 32 defaults)", state0),
        _run_trial("N=20",  state0, N=20),
        _run_trial("N=50",  state0, N=50),
        _run_trial("Qf=30", state0, Qf=30),
        _run_trial("Qf=100",  state0, Qf=100),
        _run_trial("S=1e-3",  state0, S=1e-3),
        _run_trial("S=1.0",  state0, S=1.0),
        _run_trial("dt=0.05",  state0, dt=0.05),
        _run_trial("dt=0.2",  state0, dt=0.2),
        _run_trial("tilt=10deg",  state0, tilt_deg=10.0),
        _run_trial("tilt=30deg",  state0, tilt_deg=30.0),
        _run_trial("Qv=10",  state0, Qv=10.0),
        _run_trial("Qv=30",  state0, Qv=30.0),
        _run_trial("Qv=50 (TUNED)",  state0, Qv=50.0),
        _run_trial("Qv=50 QF=50",  state0, Qv=50.0, Qf=50),
        _run_trial("Qv=50, Qf=100", state0, Qv=50, Qf=100),
        _run_trial("Qv=80,", state0, Qv=80,),
        _run_trial("Qv=100,", state0, Qv=100)
    ]

    print(f"{'Config':<28}{'landed':>8}{'pos_err[m]':>12}{'touchdown[m/s]':>16}{'fuel[kg]':>10}")
    for t in trials:
        flag = "" if t["landed"] else "   <-- NOT a real landing (ran out of fuel mid-air)"
        print(f"{t['label']:<28}{str(t['landed']):>8}{t['position_error_m']:>12.2f}"
              f"{t['touchdown_speed_mps']:>16.2f}{t['fuel_used_kg']:>10.2f}{flag}")

    return trials


def experiment_d_guided_landing():
    """
    The actual fix for Part 1's structural ceiling: a HARD terminal
    constraint (own soft-landing box - pos_err<=2m, touchdown<=1m/s
    - as smth the predicted final step must satisfy, not just smth low-cost)
    combines with a horizon that SHRINKS every real step, so the box applies
    at a real, fixed moment in time rather than being endlessly deferred.

    N=240 (24s @ dt=0.1) was found via a separate static feasibility
    probe (single open-loop solve, no closed loop): below ~15s total,
    the maneuver is INFEASIBLE outright (not enough time to kill 50m/s
    of fall AND correct 100m/50m of lateral offset withing 20deg tilt limit)
    - 240 gives comfortable margin. eps_v is deliberately
    tighter (0.15) than the literal 1m/s target, to absorb the gap
    between what the solver PREDICTS at its own horizon's end and what actually
    happens in reality (only the first control of every solve
    is ever applied - classic receding-horizon re-planning drift).
    """
    params = VehicleParams()
    weights = MpcWeights()
    state0 = nominal_scenario()

    _, _, summary = run_mpc(
        state0, params, weights, N=240, dt=0.1,
        shrink_horizon_to=10, eps_r=2.0, eps_v=0.15
    )

    meets_bar = summary["position_error_m"] <= 2.0 and summary["touchdown_speed_mps"] <= 1.0
    print(f"{'Metric':<28}{'MPC (guided descent)':>22}")
    for key, label in (("position_error_m", "Position error [m]"),
                       ("touchdown_speed_mps", "Touchdown speed [m/s"),
                       ("fuel_used_kg", "Fuel used [kg]"),
                       ("solver_failures", "Solver failures")):
        print(f"{label:<28}{summary[key]:>22f}")
    print(f"\n Meets par. 7.1 soft-landing bar (<=2m, <=1m/s): {meets_bar}")
    return summary