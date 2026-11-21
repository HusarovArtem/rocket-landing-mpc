import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from numpy.ma.core import shrink_mask

from experiments.runners import run_pid, run_mpc
from rocket_landing.controllers.mpc import MpcWeights
from rocket_landing.params import VehicleParams
from scripts.steps.check_step32 import weights


def _state0_for(r0, vz0, params):
    """
    r0 is a single scalar "how far to the hide" - x0=r0, y0.5*r0,
    same ratio as nominal() scenario()'s (100, 50) - so the grid
    stays 2D (r0 x vz0) and plottable as the simple heatmap, instead
    of a 3D (x0, y0, z0) grid that would need its own visualization story
    """
    return np.array([r0, 0.5 * r0, 500.0, 0.0, 0.0, vz0, params.initial_mass])


def _meets_soft_landing_bar(summary):
    return (bool(summary["landed"])
            and summary["position_error_m"] <= 2.0
            and summary["touchdown_speed_mps"] <= 1.0)

def run_initial_condition_grid(r0_values, vz0_values):
    """
    Experiment B (step 46): runs BOTH PID and MPC (step 47's guided-
    descent config, NOT the step-32 placeholders across every (ro, vz0)
    combination, classifying each against the soft-landing bar

    r0_values MUST not inclise 0.0 - see the degeneracy note above.
    """
    assert 0.0 not in r0_values, "r0=0 hits the NLP degeneracy - see note above, exclude it"

    params = VehicleParams()
    weights = MpcWeights()

    results = {"pid": {}, "mpc": {}}
    for r0 in r0_values:
        for vz0 in vz0_values:
            state0 = _state0_for(r0, vz0, params)

            _, _, pid_summary = run_pid(state0, params)
            results["pid"][(r0, vz0)] = {
                "ok": _meets_soft_landing_bar(pid_summary),
                "summary": pid_summary
            }

            t0 = time.perf_counter()
            _, _, mpc_summary = run_mpc(
                state0, params, weights, N=240, dt=0.1,
                shrink_horizon_to=10, eps_r=2.0, eps_v=0.15
            )

            wall_time_s = time.perf_counter() - t0
            results["mpc"][(r0, vz0)] = {
                "ok": _meets_soft_landing_bar(mpc_summary),
                "summary": mpc_summary,
                "wall_time_s": wall_time_s
            }

            print("r0=%6.1f vz0=%6.1f | PID ok=%-5s pos=%7.2f touch=%6.2f | "
                  "MPC ok=%-5s pos=%7.3f touch=%6.3f fails=%d (%.1fs)" % (
                      r0, vz0,
                      results["pid"][(r0, vz0)]["ok"], pid_summary["position_error_m"],pid_summary["touchdown_speed_mps"],
                      results["mpc"][(r0, vz0)]["ok"], mpc_summary["position_error_m"],mpc_summary["touchdown_speed_mps"],
                      mpc_summary["solver_failures"], wall_time_s))

    return results

def plot_success_failure_map(results, r0_values, vz0_values, save_path="expreriment_b_map.png"):
    """
    Two side-by-side grids (PID, MPC) - this is a STATUS encoding
    (success/failure), not a mignitude/sequential one, so per the
    dataviz skill's status palette (fixed, never themed): good=#0ca30c,
    critical=#d03b3b - and a status color never carries meaning alone,
    so every cell ALSO  gets a text label ("OK"/"FAIL"), not color only.
    """
    GOOD = "#0ca30c"
    CRITICAL = '#d03b3b'
    cmap = ListedColormap([CRITICAL, GOOD])

    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True, facecolor="#fcfcfb")
    for ax, key, title in ((axes[0], "pid", "PID"), (axes[1], "mpc", "MPC (guided descent, step 47)")):
        grid = np.array([[1.0 if results[key][(r0, vz0)]["ok"] else 0.0
                          for r0 in r0_values] for vz0 in vz0_values])
        ax.imshow(grid, cmap=cmap, vmin=0, vmax=1, aspect="auto", origin="lower")
        for yi in range(len(vz0_values)):
            for xi in range(len(r0_values)):
                ax.text(xi, yi, "OK" if grid[yi, xi] else "FAIL",
                        ha="center", va="center", color="#0b0b0b", fontsize=9)
        ax.set_xticks(range(len(r0_values)), [f"{r0:.0f}" for r0 in r0_values])
        ax.set_yticks(range(len(vz0_values)), [f"{vz0:.0f}" for vz0 in vz0_values])
        ax.set_xlabel("r0 [m] (lateral offset)")
        ax.set_title(title)
    axes[0].set_ylabel("vz0 [m/s] (initial fall speed)")
    fig.suptitle("Experiment B: soft-landing success/failure (pos<=2m, speed<=1m/s)")
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    print(f"Saved map to {save_path}")


def experiment_b_grid_map():
    # r0=0 deliberately excluded - see dependency note above. Grid kept
    # modest (16 points x 2 controllers = 32 runs) given how MPC
    # closed-loop solves are near the low end of r0 - widen later once
    # you have seen how long the default grid actually takes on your machine.
    r0_values = [10.0, 75.0, 150.0, 200.0]
    vz0_values = [-30.0, -45.0, -60.0, -80.0]

    results = run_initial_condition_grid(r0_values, vz0_values)
    plot_success_failure_map(results, r0_values, vz0_values)
    return results
