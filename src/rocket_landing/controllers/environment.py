import numpy as np


class Wind:
    """
    Simplified wind model: a CONSTANT acceleration disturbance acting on
    the rocket, idependent of its velocity or orientation.

    This is deliberately NOT a physically complete wind/drag model - a
    real crosswinf acts through aerodynamic drag, which depends on the
    rocket's velocity RELATIVE to the air, air density, cross-sectional
    area, and a drag coefficient - none of which exist in this project
    yet (F_D=0 everywhere, see docs/assumptions.md: full aerodynamic
    drag is the stretch goal in the step 49). Until then, "wind" here is
    modeled as a direct, constant acceleration term - exactly like
    gravity - standing in for "some external disturbance the vehicle has
    to compensate for". That's enough to test PID/MPC robustness without
    pretending to have solved aerodynamics early.
    """

    def __init__(self, ax=0.0, ay=0.0, az=0.0):
        self.ax = ax   # m/s^2
        self.ay = ay   # m/s^2
        self.az = az   # m/s^2 (usually 0  wind is normally horizontal)


    def acceleration(self):
        return np.array([self.ax, self.ay, self.az])