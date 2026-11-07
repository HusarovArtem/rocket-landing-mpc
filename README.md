<div align="center">

# 🚀 rocket-landing-mpc

### Autonomous Rocket Landing via Model Predictive Control

*A 3D returning-first-stage simulator, a PID baseline, and an MPC controller — built from scratch, one tested step at a time.*

![status](https://img.shields.io/badge/status-in%20progress-yellow)
![phase](https://img.shields.io/badge/phase-C%20%7C%20PID%20baseline-blue)
![progress](https://img.shields.io/badge/progress-27%2F57%20steps-orange)
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

**27 / 57 steps done — Phase C (PID baseline) in progress.**

```
Phase A  Setup & housekeeping        ████████████████████ 8/8   ✅
Phase B  Physics & simulator         ████████████████████ 14/14 ✅
Phase C  Baseline PID controller     ██████████████░░░░░░ 5/7   🔧 current
Phase D  Transition to MPC (CasADi)  ░░░░░░░░░░░░░░░░░░░░ 0/10
Phase E  Experiments & robustness    ░░░░░░░░░░░░░░░░░░░░ 0/11
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

<details open>
<summary><b>🔧 Phase C — Baseline PID controller</b> (5/7, current phase)</summary>

- [x] Landing target definition (`reference.py`): pad at origin, zero velocity
- [x] Simple PID on vertical velocity (`controllers/pid.py`)
- [x] Closed-loop PID test (vertical motion only): stable touchdown at -8.06 m/s, not yet soft but no divergence
- [x] Extend PID to horizontal position (x, y): `pid_x`/`pid_y` on position error, steering thrust tilt angles
- [x] Closed-loop simulation (`simulate_pid` in `simulation.py`): state → 3 PIDs → physics; stable on both axes (x/y overshoot, vz=-9.59 m/s at touchdown, not yet soft)
- [ ] First landing metrics (`metrics.py`) ← **next step**
- [ ] First full PID landing, plotted and evaluated

</details>

<details>
<summary><b>⏳ Phase D — Transition to MPC (CasADi)</b> (0/10)</summary>

- [ ] Symbolic dynamics in CasADi
- [ ] CasADi RK4 discretization (independent of the simulator's integrator)
- [ ] Cost function as CasADi expressions
- [ ] NLP construction: decision variables + dynamics constraints
- [ ] State & actuator constraints
- [ ] Single open-loop NLP solve (trajectory preview)
- [ ] Receding-horizon MPC loop
- [ ] Warm-start from the previous solution
- [ ] Solver-failure handling
- [ ] First full nominal MPC landing + plots

</details>

<details>
<summary><b>⏳ Phase E — Experiments and robustness</b> (0/11)</summary>

- [ ] Standardized scenarios & metrics for PID and MPC
- [ ] Migrate parameters to `configs/*.yaml`
- [ ] Experiment A — nominal PID vs MPC comparison
- [ ] Add wind (`environment.py`), repeat experiment
- [ ] Initial velocity/mass error variants
- [ ] MPC-vs-"real" model mismatch (thrust/Isp)
- [ ] Experiment B — initial-condition success/failure map
- [ ] Experiment D — MPC parameter sweep (horizon, weights, frequency)
- [ ] Experiment E — Monte Carlo campaign
- [ ] *(stretch)* Aerodynamic drag
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
