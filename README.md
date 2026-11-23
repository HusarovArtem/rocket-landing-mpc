<div align="center">

# 🚀 rocket-landing-mpc

### Autonomous Rocket Landing via Model Predictive Control

*A 3D returning-first-stage simulator, a PID baseline, and an MPC controller — built from scratch, one tested step at a time.*

![status](https://img.shields.io/badge/status-in%20progress-yellow)
![phase](https://img.shields.io/badge/phase-D%20%7C%20MPC%20transition-blue)
![progress](https://img.shields.io/badge/progress-48%2F57%20steps-orange)
![python](https://img.shields.io/badge/python-3.14-3776AB)
![solver](https://img.shields.io/badge/NLP-CasADi%20%2B%20IPOPT-informational)

</div>

---

## 📖 Overview

This is an engineering thesis (*praca dyplomowa*) project: land a simulated rocket's returning first stage, autonomously, in 3D — first with a classic **PID** controller, then with a **Model Predictive Control (MPC)** controller built on **CasADi + IPOPT** — and compare the two under disturbances (wind, model mismatch, initial-condition errors).

> **Research question:** To what extent does MPC improve the accuracy and quality of a returning rocket's autonomous landing compared to a baseline PID controller, under limited thrust, model uncertainty, and disturbances?

It's deliberately **not** a 6-DoF, aerodynamically exact rocket simulator — see [Scope & Simplifications](#-scope--simplifications) below. The goal is a physically credible, fully-tested, reproducible GNC (Guidance, Navigation & Control) pipeline, not a maximally realistic one.

## 🧭 Table of contents

- [Overview](#-overview)
- [Progress](#-progress)
- [Architecture](#-architecture)
- [Tech stack](#-tech-stack)
- [Scope & simplifications](#-scope--simplifications)
- [Running it](#-running-it)
- [Where to read more](#-where-to-read-more)

## 📊 Progress

**48 / 57 steps done — Phase D (MPC transition) complete, Phase E core experiments complete (only stretch goals left).**

```
Phase A  Setup & housekeeping        ████████████████████ 8/8   ✅
Phase B  Physics & simulator         ████████████████████ 14/14 ✅
Phase C  Baseline PID controller     ████████████████████ 7/7   ✅
Phase D  Transition to MPC (CasADi)  ████████████████████ 10/10 ✅
Phase E  Experiments & robustness    ████████████████░░░░ 9/11  🔧 current
Phase F  Thesis writing & finish     ░░░░░░░░░░░░░░░░░░░░ 0/7
```

> Checkboxes below are the live source of truth for *what's actually been verified*, not just written — every item was manually run and checked before being ticked. Full step-by-step log with verification details: [`PROGRESS.md`](PROGRESS.md) (Polish, canonical) / [`PROGRESS.en.md`](PROGRESS.en.md) (English snapshot).

<details open>
<summary><b>✅ Phase A — Setup and housekeeping</b> (8/8)</summary>

- [x] Repo cleanup, old PyCharm boilerplate removed
- [x] Original thesis description preserved for reference
- [x] `pyproject.toml` with numpy / scipy / matplotlib / casadi / pytest
- [x] Installation smoke test (numpy, matplotlib, pytest)
- [x] CasADi + IPOPT installed and sanity-checked on a toy NLP
- [x] Package skeleton (`src/rocket_landing/`)
- [x] Cross-session progress tracking set up
- [x] Physical model assumptions documented

</details>

<details open>
<summary><b>✅ Phase B — Physics and simulator</b> (14/14, no controller yet)</summary>

- [x] Vehicle parameters (mass, thrust limits, Isp, max tilt angle)
- [x] Free-fall gravity dynamics
- [x] Thrust-direction unit vector `d(alpha, beta)`
- [x] Full dynamics: gravity + thrust + mass consumption
- [x] Euler integrator
- [x] Manual free-fall verification vs. hand calculation
- [x] Manual constant-thrust verification vs. hand calculation
- [x] First pytest suite (`test_dynamics.py`)
- [x] RK4 integrator
- [x] RK4 vs Euler accuracy test (RK4 ~1000x more accurate)
- [x] Ground-contact detection (stop simulation at `z <= 0`)
- [x] Thrust & tilt-angle clamping (`actuators.py`)
- [x] Height/velocity/mass time-series plots
- [x] First 3D trajectory plot with lateral drift

</details>

<details>
<summary><b>✅ Phase C — Baseline PID controller</b> (7/7)</summary>

- [x] Landing target definition (`reference.py`): pad at origin, zero velocity
- [x] Simple PID on vertical velocity (`controllers/pid.py`)
- [x] Closed-loop PID test (vertical motion only): stable touchdown at -8.06 m/s, not yet soft but no divergence
- [x] Extend PID to horizontal position (x, y): `pid_x`/`pid_y` on position error, steering thrust tilt angles
- [x] Closed-loop simulation (`simulate_pid` in `simulation.py`): state → 3 PIDs → physics; stable on both axes (x/y overshoot, vz=-9.59 m/s at touchdown, not yet soft)
- [x] First landing metrics (`metrics.py`): position error ≈30.6 m, touchdown speed ≈10.8 m/s, fuel used ≈1772 kg
- [x] First full PID landing, plotted and evaluated (`plot_full_pid_landing()`): 3D path visibly overshoots the pad, time series confirm metrics

</details>

<details>
<summary><b>✅ Phase D — Transition to MPC (CasADi)</b> (10/10)</summary>

- [x] Symbolic dynamics in CasADi (`casadi_dynamics()` in `controllers/mpc.py`): independent rewrite, numerically matches `dynamics.py` exactly on hover + tilted test cases
- [x] CasADi RK4 discretization (`casadi_rk4_step()`, independent of the simulator's integrator): matches `rk4_step()` exactly on hover + tilted test cases, different `dt` values
- [x] Cost function as CasADi expressions (`stage_cost()`, `terminal_cost()` in `controllers/mpc.py`): manually verified against hand-worked values, `stage_cost` ≈ 50230.25, `terminal_cost` = 61.8
- [x] NLP construction: decision variables + dynamics constraints (`build_mpc_nlp()` in `controllers/mpc.py`, CasADi `Opti`): fully-pinned test case solved by IPOPT matches a manual `casadi_rk4_step` chain exactly (max diff 0.0)
- [x] State & actuator constraints (`add_state_and_actuator_constraints()` in `controllers/mpc.py`): thrust/tilt bounds correctly pin to their limits under a maximizing objective; forced through-the-ground and over-burn scenarios correctly reported INFEASIBLE by IPOPT
- [x] Single open-loop NLP solve / trajectory preview (`solve_open_loop_trajectory()` in `controllers/mpc.py`): wires cost + constraints + initial condition together, IPOPT finds an optimal, fully constraint-respecting solution; revealed that today's placeholder weights don't yet produce braking behavior (expected — see [`rozdzial_06_mpc.md`](thesis/docx/rozdzial_06_mpc.md) §6.4; turned out NOT to be a weight-tuning problem at all — see §6.11 / step 47)
- [x] Receding-horizon MPC loop (`simulate_mpc()` in `controllers/mpc.py`): re-solves the NLP from the real current state every step, applies only `U_opt[:, 0]` via the simulator's own `rk4_step()`/`clamp_control()` (independent of the controller's internal CasADi model); verified over 5 real steps that the applied control genuinely changes step to step (plan is recomputed, not cached)
- [x] Warm-start from the previous solution (`simulate_mpc()` feeds its own shifted solution forward as the next solve's initial guess): verified over 8 real steps — identical iteration count on step 0 (no prior solution either way), drastically fewer IPOPT iterations from step 1 onward (92 vs 349 total), nearly identical final states
- [x] Solver-failure handling (`simulate_mpc()` catches IPOPT `RuntimeError`, logs a warning, falls back to the last applied control or a hover default): verified on a normal scenario (0 failures) and a deliberately infeasible-from-the-start one (failure logged, exact hover-thrust fallback applied, simulation did not crash)
- [x] First full nominal MPC landing + plots (`plot_full_mpc_landing()` in `scripts/plot_profile.py`): full closed loop runs start-to-finish without crashing, but with today's un-tuned weights the landing is **not soft** — touchdown at ≈49.39 m/s (worse than the PID baseline's 10.8 m/s), 30/83 real steps needed the step-38 fallback near the ground, ~10.2 avg IPOPT iterations/step thanks to warm-starting. `vz` accelerates downward at first (position-error term dominates), then starts braking too late once position error shrinks near the pad — confirming, over a full flight, the step 35/36 finding. **Resolved in step 47** (§6.11): not a weight-tuning issue — the soft terminal cost let the controller defer landing indefinitely; a hard terminal constraint + shrinking horizon fixes it (0.95 m/s touchdown, 0.77m position error, 0 solver failures), prototype-verified, integration pending

</details>

<details open>
<summary><b>🔧 Phase E — Experiments and robustness</b> (7/11, current phase)</summary>

- [x] Standardized scenarios & metrics for PID and MPC (`experiments/runners.py`: `nominal_scenario()`, `run_pid()`/`run_mpc()` returning the same `landing_summary` format for both): verified to exactly reproduce the already-confirmed PID (steps 27-29) and MPC (step 39) results
- [x] Migrate parameters to `configs/*.yaml` (`VehicleParams`/`MpcWeights` gained an optional `config_path` arg + `_load_from_yaml()`, defaulting to unchanged behavior): verified 14/14 fields identical between hardcoded defaults and `configs/vehicle_nominal.yaml`/`configs/mpc_weights_default.yaml`
- [x] Experiment A — nominal PID vs MPC comparison (`experiments/experiment_a.py`): side-by-side table confirms PID currently wins on every metric except fuel (position error 30.58 vs 57.39 m, touchdown speed 10.83 vs **49.39 m/s**, fuel 1772 vs 724 kg) — an honest baseline *before* the step 47 fix, not a final verdict (post-fix MPC touchdown speed is 0.95 m/s, well under PID's 10.83 m/s — see step 47 below)
- [x] Add wind (`environment.py`: `Wind` class, constant disturbance acceleration — wired into the REAL simulator only, never the MPC's internal model): verified against analytic kinematics in isolation, then via `experiment_a_wind()` — PID position error 30.58→16.32 m (wind coincidentally cancels PID's existing overshoot bias, not a general improvement), MPC 57.39→66.74 m, fuel essentially unchanged for both (purely lateral disturbance)
- [x] Initial velocity/mass error variants — Experiment C (`experiments/experiment_c.py`: `scenario_with_initial_error()`, `experiment_c_initial_error()`): rocket actually starts falling 10 m/s faster (vz=-60) and 500 kg lighter (24500 kg) than the nominal scenario assumed, controller sees the real starting state correctly — PID position error 30.58→11.08 m but touchdown speed 10.83→**25.01 m/s** (less time to brake), MPC 57.39→56.78 m / 49.39→54.29 m/s; a genuine speed/accuracy trade-off, not an inconsistency
- [x] MPC-vs-"real" model mismatch (`experiment_a_model_mismatch()`): real engine 10% weaker (360 vs 400 kN) and 10% less efficient (Isp 270 vs 300s) than the MPC's internal model assumes — PID barely changes (30.58→31.05 m, 10.83→10.54 m/s, since it has no engine model to be wrong about, just +200 kg fuel from worse Isp), MPC's touchdown speed gets **11.6% worse** (49.39→55.14 m/s) specifically because it plans braking assuming thrust it doesn't actually have — textbook plant-model mismatch
- [x] Experiment D — MPC parameter sweep + structural fix, **both parts verified in the real codebase** (`check_step47_part1.py`, `check_step47_part2.py`). Part 1 (`metrics.py`'s new `landed` flag + `experiments/experiment_d.py` — all 18 sweep rows reproduce the prototype exactly): swept the 5 parameters the thesis plan names (horizon, terminal weight, control-change penalty, control frequency, tilt limit) one at a time — **none** of them meaningfully improve real touchdown speed (stays in the 45-68 m/s "crash" band); the one parameter that does (`Qv`, velocity-error weight) hits a ceiling where pushing further just makes the sim run out of fuel hovering instead of landing (that's what the new `landed` flag catches). Part 2 — the actual fix: a **hard terminal constraint** + a **shrinking horizon** (instead of relying on cost weights alone, exactly what `plan_original.md` itself warned was needed) achieves a genuine soft landing with *unchanged* weights: **0.953 m/s touchdown, 0.772m position error, 0 solver failures** — the first config in the whole project to meet the §7.1 soft-landing bar. Caught two real transcription bugs along the way (`<=` misplaced inside `ca.sumsqr()`; a typo'd duplicate `shrink_horizon_to` parameter that silently disabled the horizon-shrinking entirely). Full derivation: [`rozdzial_06_mpc.md`](thesis/docx/rozdzial_06_mpc.md) §6.11
- [x] Experiment B — initial-condition success/failure map (`experiments/experiment_b.py`, `check_step46.py`), 4×4 grid (r0 ∈ {10,75,150,200}m × vz0 ∈ {-30,-45,-60,-80}m/s, 16 points × 2 controllers = 32 full closed-loop runs, ~123 min total): **PID meets the §7.1 soft-landing bar in 0/16 points (never)** — touchdown speed always far over 1 m/s regardless of starting condition. **MPC meets it in 7/16 (44%)**, but success is concentrated close to the pad — all 4 speeds succeed at r0=10m, only the slowest succeeds by r0=150m, and **none** succeed at r0=200m. Answers the open question from step 47: the fixed config (N=240, eps_v=0.15) tuned on one nominal scenario does **not** generalize uniformly — it degrades with lateral distance, especially combined with fall speed. Also corrected an earlier methodological error: a one-shot *open-loop* feasibility probe at the hardest corner (r0=200, vz0=-80) had suggested the config generalizes (feasible in 11.6s) — the real *closed-loop* run at that same point fails worst of all (71m error, 14.7 m/s touchdown), showing open-loop feasibility checks don't predict closed-loop performance. One genuine solver infeasibility occurred across the whole grid (r0=150, vz0=-80, step 40 — fallback applied, no crash). Full table: [`rozdzial_08_09_10_dodatki_szkielet.md`](thesis/docx/rozdzial_08_09_10_dodatki_szkielet.md) §8.2
- [x] Experiment E — Monte Carlo campaign (`experiments/experiment_e.py`, `check_step48.py`), 20 random trials (seed=48, random start point + direction + fall speed + ±10% random engine thrust/Isp error, ~6.2 hours total): **PID meets the §7.1 bar in 0/20 (0%), MPC in 3/20 (15%)**. The headline finding: of MPC's 17 failures, **45% (9/20) are not crashes at all — they're a "ran out of fuel just short of the ground" artifact** (near-zero touchdown speed but `landed=False`), and every single one of those 9 trials — and in fact all 12 trials with `isp_frac < 1.0` — has the engine *less fuel-efficient* than MPC's internal model assumes. Since Isp directly sets fuel burn rate, an overestimated Isp means the real rocket burns fuel faster than planned, so it runs dry right before satisfying the step-47 hard terminal constraint and falls back to hovering until fuel hits zero. Genuine crashes (4/20) need high fall speed *and* a weak engine; near-misses (4/20) are mostly far-from-pad cases, consistent with step 46. Conclusion: the step-47 structural fix is solid under a matched model (0 artifacts in step 46's 16 no-mismatch trials) but specifically fragile to Isp underestimation, not thrust error or distance. Full breakdown: [`rozdzial_08_09_10_dodatki_szkielet.md`](thesis/docx/rozdzial_08_09_10_dodatki_szkielet.md) §8.5
- [ ] *(stretch)* Aerodynamic drag ← **next step (optional — decide: continue stretch goals or move to Phase F)**
- [ ] *(stretch)* Orientation / body-tilt extension

</details>

<details>
<summary><b>⏳ Phase F — Thesis writing and finish</b> (0/7)</summary>

- [ ] Numerical verification & constraint-violation audit
- [ ] Methodology & mathematical model chapter
- [ ] MPC formulation & implementation chapter
- [ ] Experiments & results chapter
- [ ] Conclusions, limitations & final README
- [ ] Presentation + demo recording + English technical report
- [ ] Defense rehearsal

</details>

> **Maintenance note (for future-me / Claude):** this checklist is updated in lockstep with `PROGRESS.md` every time a step is checked off there — if you're reading this and it looks stale, re-sync it against `PROGRESS.md` first.

## 🏗️ Architecture

```
rocket-landing-mpc/
├── src/rocket_landing/
│   ├── params.py          # vehicle constants (mass, thrust, Isp, tilt limit)
│   ├── dynamics.py        # equations of motion + thrust-direction vector
│   ├── integrators.py     # Euler, RK4
│   ├── actuators.py       # thrust / tilt-angle clamping
│   ├── simulation.py      # fixed-control simulation loop + landing detection
│   ├── reference.py       # landing target (position, velocity)
│   ├── controllers/       # pid.py, (later) mpc.py
│   ├── environment.py     # wind etc. (Phase E)
│   └── metrics.py         # landing error, fuel use, violations (Phase C/E)
├── scripts/                # manual plotting scripts
├── tests/                  # pytest suite
├── experiments/             # PID vs MPC experiment runners (Phase E)
├── configs/                 # YAML configs (Phase E)
├── docs/assumptions.md      # physical model assumptions (+ .en.md)
└── thesis/plan_original.md  # full original thesis brief (+ .en.md)
```

Design principle (from the original brief): **dynamics never depends on the controller, and the controller never depends on visualization** — so PID can be swapped for MPC without touching the simulator.

## 🛠️ Tech stack

| Tool | Role |
| --- | --- |
| **Python 3.14** | implementation language |
| **NumPy / SciPy** | numerics, integration |
| **CasADi + IPOPT** | symbolic modeling & nonlinear optimization for MPC |
| **Matplotlib** | time-series & 3D trajectory plots |
| **pytest** | automated physics/controller/metrics tests |

## 🎯 Scope & simplifications

**Included:** 3D translational dynamics, variable mass (fuel burn via Isp), bounded thrust, bounded thrust-tilt angle, gravity.

**Explicitly excluded (by design, not yet):**
- No aerodynamic drag (stretch goal, step 49)
- No 6-DoF body-orientation dynamics — thrust direction is steered directly as a unit vector, no attitude control (stretch goal, step 50)
- No propellant slosh, body flexibility, or engine turbomachinery
- No real planet physics (no Earth rotation/Coriolis, no real atmosphere) — flat local frame, constant gravity
- Vehicle parameters are a plausible **educational "toy" rocket**, not a real one (e.g. not Falcon 9 data)

Full rationale: [`docs/assumptions.md`](docs/assumptions.md) ([English](docs/assumptions.en.md)).

## ▶️ Running it

```bash
source .venv/bin/activate
pytest tests/ -v
```

(Plotting scripts under `scripts/` are run individually, e.g. `python scripts/plot_profile.py`, and open a Matplotlib window.)

## 📚 Where to read more

| File | Contents |
| --- | --- |
| [`PROGRESS.md`](PROGRESS.md) | Canonical, live step-by-step log (Polish) |
| [`PROGRESS.en.md`](PROGRESS.en.md) | English snapshot of the above |
| [`thesis/plan_original.md`](thesis/plan_original.md) | Full original thesis brief: math, architecture, experiment plan, literature (Polish) |
| [`thesis/plan_original.en.md`](thesis/plan_original.en.md) | English translation of the above |
| [`docs/assumptions.md`](docs/assumptions.md) | Physical model assumptions (Polish) |
| [`docs/assumptions.en.md`](docs/assumptions.en.md) | English translation of the above |

---

<div align="center">

*A thesis project in Guidance, Navigation & Control — built and verified one step at a time.*

</div>
