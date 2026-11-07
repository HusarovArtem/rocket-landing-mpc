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

def simulate_pid(state0, pid_vz, pid_x, pid_y, params, dt, max_steps=100000):
    """
    Full 3D closed-loop landing attempt: the vertical PID (pid_vz) still
    controls vz exactly like in step 25 (thrust = hover_thrust + correction)
    but now TWO more PID controllers steer the rocket sideways, back toward
    x=0, y=0 (the landing pad), by commanding the tilt angles alpha and beta.

    Key diff from pid_vz: pid_x and pid_y are given a POSITION error target_x
    - x), not a velocity error. Why this works without a separate "kill the
    horizontal velocity" step: look at what the D term of a PID on POSITION
    actually computes; derivative = (error_now - error_prex) / dt
                                  = d(target - x)/dt - -dx/dt = -vx

   So kd_x * derivative is really "kd_x * (-vx)" - a term that AUTOMACALLY
   pushes against whatever horizontal velocity currently exists, exactly
   like a shock absorber / damper. A plain PID on position is secretly
   also damping velocity, with no extra code needed - this is why we
   don't need a 4th controller.

   Converting a desired horizontal FORCE into an ANGLE: thrust_direction
   makes thrust's x-component equal to `thrust * sin(alpha) * cos(beta)`.
   For the small angles  we're restricted to anyway (max_tilt_angle is only
   ~20 degrees), sin(x) ~= x and cos(x) ~= 1, so thrust_x ~= thrust * alpha
   => alpha ~= desired_Fx / thrust deliberate simplification worth remembering
   if later testing reveals coupling issues between the lateral and vertical motion
   """
    times = [0.0]
    states = [np.array(state0, dtype=float)]

    for i in range(max_steps):
        current_state = states[-1]
        x, y, z = current_state[0:3]
        vz = current_state[5]
        mass = current_state[6]

        # --- vertical control (unchanged from step 25) ---
        vz_error = 0.0 - vz
        vz_correction = pid_vz.compute(vz_error, dt)
        thrust = mass * params.g + vz_correction

        # -- horizontal control (new) ---
        x_error = 0.0 - x
        y_error = 0.0 - y
        desired_fx = pid_x.compute(x_error, dt)
        desired_fy = pid_y.compute(y_error, dt)


        # Guard against dividing by a tiny/zero thrust (shouldn't happen
        # given min_thrust > 0, but thrust here is PRE-clamp - cheap
        # safety net before the division below
        safe_thrust = max(thrust, params.min_thrust)
        alpha = desired_fx / safe_thrust
        beta = desired_fy / safe_thrust

        control = clamp_control(np.array([thrust, alpha, beta]), params)

        next_state = rk4_step(current_state, control, params, dt)
        times.append(times[-1] + dt)
        states.append(next_state)

        if next_state[2] <= 0.0:
            break

        if next_state[6] <= params.dry_mass:
            break

    return np.array(times), np.array(states)
