import numpy as np


def landing_position_error(final_state, target):
    """
    "how many meters away from the pad did we
    actually land?" - straight-line distance, counting x, y AND x (not
    just sideways drift).

    final_state: ONE state (7 numbers: [x, y, z, vx, vy, vz, m] - you
        pass in states[-1], the touchdown state.
    target: a LandingTarget (reference.py). We read target.position
        ([0,0,0], the pad) out of it
    """
    position = final_state[0:3]
    error_vector = position - target.position
    return float(np.linalg.norm(error_vector))


def landing_speed(final_state):
    """
     "how fast (in any direction combined) were we moving instant we touched down?"
    """
    velocity = final_state[3:6]
    return float(np.linalg.norm(velocity))


def  fuel_used(initial_mass, final_state):
    """
    "How many kg of fuel did the whole flight burn?"
    """
    final_mass = final_state[6]
    return float(initial_mass - final_mass)

def landing_summary(times, states, target, initial_mass):
    """
    Convenience wrapper - instead of calling all 3 functions above
    separately every time, call this ONE function and get everything
    back together in a plain Python dict ( a dict is just labelled boxes:
    {"label": value, "label2": value2, ...} - here it`s earlier to read
    printed output than 4 separate bare numbers).
    """
    final_state = states[-1]
    return {
        "flight_time_s": float(times[-1]),
        "position_error_m": landing_position_error(final_state, target),
        "touchdown_speed_mps": landing_speed(final_state),
        "fuel_used_kg": fuel_used(initial_mass, final_state)
    }