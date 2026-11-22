
import time

import matplotlib.pyplot as plt
import numpy as np

from experiments.runners import run_mpc, run_pid
from rocket_landing.controllers.mpc import MpcWeights
from rocket_landing.params import VehicleParams

SEED = 48  # fixed seed so results can be reproduced later
N_TRIALS = 20

def _random_trial_inputs(rng, model_params):
    """ Picks one random start and one random engine error """
    r0 = rng.uniform(10.0, 200.0)  # how far to the side from the starting point
    theta = rng.uniform(0.0, 2 * np.pi)  # random angle 360 degrees (which side)
    # Negative vz means falling down, positive means going up
    vz0 = rng.uniform(-80.0, -30.0)  # random speed of fall on start (-80 to -30 m/s)
    # random value of the engine thrust error (from 0.9 to 1.1)
    thrust_frac = rng.uniform(0.9, 1.1)  # 90% to 110% of what MPC expects from the engine
    # how much thrust the engine gives per each kg of burnt fuel
    isp_frac = rng.uniform(0.9, 1.1)  # 90% to 110% of what MPC expects

    state0 = np.array([
        r0 * np.cos(theta), r0 * np.sin(theta), 500.0,
        0.0, 0.0, vz0, model_params.initial_mass
    ])

    real_params = VehicleParams()
    real_params.max_thrust = thrust_frac * model_params.max_thrust
    real_params.isp = isp_frac * model_params.isp

    return state0, real_params, dict(r0=r0, theta=theta, vz0=vz0,
                                      thrust_frac=thrust_frac, isp_frac=isp_frac)

def _meets_soft_landing_bar(summary):
    """ Same check as we used in step 46 """
    return (bool(summary["landed"])
            and summary["position_error_m"] <= 2.0
            and summary["touchdown_speed_mps"] <= 1.0)

def run_monte_carlo_trials(n_trials=N_TRIALS, seed=SEED):
    """
    Runs n_trials random PID + MPC flights. Returns a list of per-trial
    result dicts (one per trial), so stats and plots can be made after.
    """
    rng = np.random.default_rng(seed)
    model_params = VehicleParams()
    weights = MpcWeights()

    trials = []
    for i in range(n_trials):
        state0, real_params, inputs = _random_trial_inputs(rng, model_params)

        # PID does not have an engine model, so it just flies on the real
        # (possibly wrong) engine directly - same pattern as step 45
        _, _, pid_summary = run_pid(state0, real_params)

        t0 = time.perf_counter()
        _, _, mpc_summary = run_mpc(
            state0, model_params, weights, N=240, dt=0.1,
            shrink_horizon_to=10, eps_r=2.0, eps_v=0.15,
            real_params=real_params,
        )
        mpc_time_s = time.perf_counter() - t0

        trial = dict(
            trial=i, inputs=inputs,
            pid_ok=_meets_soft_landing_bar(pid_summary), pid_summary=pid_summary,
            mpc_ok=_meets_soft_landing_bar(mpc_summary), mpc_summary=mpc_summary,
            mpc_time_s=mpc_time_s,
        )
        trials.append(trial)

        print("trial %2d | r0=%6.1f vz0=%6.1f thrust=%.2fx isp=%.2fx | "
              "PID ok=%-5s pos=%7.2f touch=%6.2f | "
              "MPC ok=%-5s pos=%7.3f touch=%6.3f fails=%d (%.1fs)" % (
            i, inputs["r0"], inputs["vz0"], inputs["thrust_frac"], inputs["isp_frac"],
            trial["pid_ok"], pid_summary["position_error_m"], pid_summary["touchdown_speed_mps"],
            trial["mpc_ok"], mpc_summary["position_error_m"], mpc_summary["touchdown_speed_mps"],
            mpc_summary["solver_failures"], mpc_time_s))

    return trials

def print_mc_stats(trials):
    """
    Prints the simple numbers a thesis table needs: success rate, and
    median/95th-percentile/max of error, speed, and compute time.
    """
    n = len(trials)
    pid_ok_count = sum(t["pid_ok"] for t in trials)
    mpc_ok_count = sum(t["mpc_ok"] for t in trials)

    mpc_pos = np.array([t["mpc_summary"]["position_error_m"] for t in trials])
    mpc_touch = np.array([t["mpc_summary"]["touchdown_speed_mps"] for t in trials])
    mpc_time = np.array([t["mpc_time_s"] for t in trials])

    pid_pos = np.array([t["pid_summary"]["position_error_m"] for t in trials])
    pid_touch = np.array([t["pid_summary"]["touchdown_speed_mps"] for t in trials])

    print("\n=== Monte Carlo summary (%d trials, seed=%d) ===" % (n, SEED))
    print("PID success rate: %d/%d (%.0f%%)" % (pid_ok_count, n, 100 * pid_ok_count / n))
    print("MPC success rate: %d/%d (%.0f%%)" % (mpc_ok_count, n, 100 * mpc_ok_count / n))
    print()
    print("%-28s %10s %10s" % ("Metric", "PID", "MPC"))
    print("%-28s %10.2f %10.2f" % ("Position error, median [m]", np.median(pid_pos), np.median(mpc_pos)))
    print("%-28s %10.2f %10.2f" % ("Position error, p95 [m]", np.percentile(pid_pos, 95), np.percentile(mpc_pos, 95)))
    print("%-28s %10.2f %10.2f" % ("Touchdown speed, median [m/s]", np.median(pid_touch), np.median(mpc_touch)))
    print("%-28s %10.2f %10.2f" % ("Touchdown speed, p95 [m/s]", np.percentile(pid_touch, 95), np.percentile(mpc_touch, 95)))
    print("%-28s %10s %10.1f" % ("MPC compute time, median [s]", "n/a", np.median(mpc_time)))
    print("%-28s %10s %10.1f" % ("MPC compute time, p95 [s]", "n/a", np.percentile(mpc_time, 95)))
    print("%-28s %10s %10.1f" % ("MPC compute time, max [s]", "n/a", np.max(mpc_time)))

def plot_mc_scatter(trials, save_path="experiment_e_scatter.png"):
    """
    One scatter plot per controller: x = position error, y = touchdown
    speed, each dot is one trial. Green circle = pass, red x = fail (two
    cues, not just color, so it still reads fine without color).
    """
    GOOD = "#0ca30c"
    CRITICAL = "#d03b3b"

    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True, facecolor="#fcfcfb")
    for ax, key, title in ((axes[0], "pid", "PID"), (axes[1], "mpc", "MPC (guided descent, step 47)")):
        ok_key = f"{key}_ok"
        summary_key = f"{key}_summary"
        pos = [t[summary_key]["position_error_m"] for t in trials]
        touch = [t[summary_key]["touchdown_speed_mps"] for t in trials]
        ok = [t[ok_key] for t in trials]

        for p, s, o in zip(pos, touch, ok):
            color = GOOD if o else CRITICAL
            marker = "o" if o else "x"
            ax.scatter(p, s, color=color, marker=marker, s=60)

        ax.axvline(2.0, color="#888888", linestyle="--", linewidth=1)
        ax.axhline(1.0, color="#888888", linestyle="--", linewidth=1)
        ax.set_xlabel("position error [m]")
        ax.set_title(title)
    axes[0].set_ylabel("touchdown speed [m/s]")
    fig.suptitle("Experiment E: Monte Carlo, %d trials (seed=%d)" % (len(trials), SEED))
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    print(f"Saved plot to {save_path}")

def experiment_e_monte_carlo():
    trials = run_monte_carlo_trials(N_TRIALS, SEED)
    print_mc_stats(trials)
    plot_mc_scatter(trials)
    return trials
