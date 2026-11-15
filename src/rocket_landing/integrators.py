import numpy as np
from scipy.differentiate import derivative

from rocket_landing.dynamics import dynamics


def euler_step(state, control, params, dt):
    """
    Advances `state` forward by one small time step `dt`, using the
    SIMPLEST possible method: Euler integration.


    The idea: we already know the INSTANTANEOUS rate of change of the
    state (that's exactly what dynamics() computes - see its docstring).
    Euler integration just assumes that rate of change stays constant for
    the whole tiny duration `dt`, and takes one straight step using it:

        state(t + dt) ~= state(t) + dt * d(state)/dt


    This is an APPROXIMATION - it's only exactly correct if the rate of
    change truly never changes during the step (which is never quite
    true, except for velocity under exactly constant acceleration - see
    the check below). The smaller `dt` is, the smaller the error, but it
    never goes away completely. That's WHY we'll add a better integrator
    (RK4) later in step 17 - this EULER one is just a fast, simole
    starting point to get the simulator running end-to-end.
    """
    derivative = dynamics(state, control, params)
    return state + dt * derivative

def rk4_step(state, control, params, dt, wind=None):
    """
    Advances 'state' forward by one step 'dt' using RK4
    (4th-order Runge-Kutta) - a much more accurate alternative
    to euler_step().

    Euler's weakness: it only looks at the derivative ONCE, at the
    START of the step, and assumes it stays constant the whole way
    RK4`s idea: instead of one sample, take FOUR samples of the
    derivative at different points across the step (start, two
    "probes" near the middle, and the end), then combine all four
    with carefully chosen step, not just its starting value - which
    is exactly what Euler was missing.
    """
    # k1: the derivative at the very start of the step (this is the
    # ONLY thing euler_Step uses)
    k1 = dynamics(state, control, params, wind)

    # k2: take a trial half-step forward using K1, then re-check the
    # derivative there - a first "probe" into the middle of the interval
    k2 = dynamics(state + dt / 2 * k1, control, params, wind)

    # k3: take another trial half-step, this time using k2's slope
    # instead of k1`s - a second, refined probe at the middle
    k3 = dynamics(state + dt / 2 * k2, control, params, wind)

    # k4: take a trial FULL step using k3, then check the derivative
    # at the (estimated) END of the interval
    k4 = dynamics(state + dt * k3, control, params, wind)

    # combine all four slope estimates with the classic
    # RK4 weights (1, 2, 2, 1) / 6 - these specific wights
    # are what make the method 4th order accurate
    return state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

