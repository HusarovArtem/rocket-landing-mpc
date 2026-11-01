from readline import parse_and_bind

import numpy as np
from fontTools.designspaceLib.types import clamp

from rocket_landing.actuators import clamp_control
from rocket_landing.integrators import rk4_step


def simulate(state0, control, params, dt, max_steps=100000):
    """
    Runs a full simulation starting from 'state0', repeatedly
    calling rk4_step with a CONSTANT 'control' (same thrust/
    alpha/beta every step), and STOPS as soon as the rocket touches
    the ground ( z <= 0 ) - instead of running forever or crashing into
    negative altitude nonsensically.

    Returns:
        times   - 1D array, one timestamp per recorded state (starts at 0)
        states  - 2D array, one row per recorded state (so states[-1] is
                  the final/landing state, stated[-1][2] is landing height)
    """
    times = [0.0]
    states = [np.array(state0, dtype=float)]

    for i in range(max_steps):
        current_state = states[-1] # latest state from states

        next_state = rk4_step(current_state, control, params, dt)
        times.append(times[-1] + dt)
        states.append(next_state) # adding new next steps to the current state

        #  z - height
        if next_state[2] <= 0.0:
            # Touched (or passed through) the ground - stop right there,
            # don't keep simulating a rocket that's already landed/crashed
            break

    return np.array(times), np.array(states)

def simulate_vertical_pid(state0, pid, params, dt, max_steps=100000):
    """
    First CLOSED-LOOP simulation: unlike simulate() (step 19), which used
    one FIXED control the whole way through, this recomputes the thrust
    at EVERY single step, by asking `pid` how far off the current
    vertical speed (vz) is from the target (0 - we want to stop, not
    fall). alpha and beta are kept at 0 the whole time - motion is
    vertical-only on purpose, exactly as this step's tittle says. Full
    3D closed-loop control (horizontal PID too) comes in step 26-27.


    Why "hover_thrust" + correction" instead of just "correction":
    gravity alone requires 'mass * g' newtons of thrust just to stand
    still (see the hover test from step 12). If the PID's output were
    used AS the thrust directly, it would have to re-learn "flight
    gravity" from scratch via the error - by instead giving it a head
    start at the hover point and only asking it to supply the REST (the
    correction needed to kill the remaining vz), the PID's job becomes
    much simpler: pure error-connection, not "also hold up the entire
    rocket". This is a common, standard PID-control trick, not specific
    to rockets.
    """
    times = [0.0]
    states = [np.array(state0, dtype=float)]

    for i in range(max_steps):
        current_state = states[-1]
        vz = current_state[5]
        mass = current_state[6]


        # Target vz is 0 (docs/assumptions.md / reference.py's
        # LandingTarget.velocity - so error = how far from a complete
        # stop we currently are. Positive errors means "falling too
        # fast", need MORE thrust"/
        error = 0.0 - vz
        correction = pid.compute(error, dt)

        hover_thrust = mass * params.g
        control = clamp_control(
            np.array([hover_thrust + correction, 0.0, 0.0]), params
        )

        next_state = rk4_step(current_state, control, params, dt)
        times.append(times[-1] + dt)
        states.append(next_state)

        if next_state[2] <= 0.0:
            break

        if next_state[6] <= params.dry_mass:
            # Ran out of fuel before landing - stop here
            break

    return np.array(times), np.array(states)

