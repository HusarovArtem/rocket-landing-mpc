import math

import yaml

class VehicleParams:
    """
    Physical constants and limits describing our educational rocker stage.
    This is NOT a real rocket. These are plausible "toy" numbers for a small
    returning first-stage, picked so that the engine CAN actually produce
    enough thrust to slow the rocket down and land softly (we will sanity-check
    that in the next couple of steps, with free-fall and hover tests).
    """

    def __init__(self, config_path=None):
        # ------ Mass properties ------------------------------------------
        # Total mass at the START of the landing burn, in KG
        # This includes the dry structure PLUS the fuel left for landing.
        self.initial_mass = 25000.0  # kg

        # Mass of the stage with ZERO fuel left (structure, engine, legs...)
        # The simulator must never let the mass drop below this value -
        # we will enforce that later as a constraint
        self.dry_mass = 22000.0 #kg

        # ------ Engine ----------------------------------------------------
        # Specific impulse: hpw efficiently the engine turns propellant mass
        # into thrust. Bigger isp = more thrust per kg of fuel burned per
        # second. Used only in the mass-flow eqation, dm/dt = -T/(isp*g0).
        self.isp=300.0 #seconds

        # Standard gravitational acceleration. This shows up only inside the
        # mass flow equation above - it is a fixed physical constant, by
        # convention always 9.80665, even on a different planet. Don`t confuse
        # rocket downward in its equations of motion.
        self.g0 = 9.80665 # m/s^2

        # The engine cannot produce more thrust than max_thrust, and (for
        # this flight phase) we also assume it cannot be commanded below
        # min_thrust - i.e. it can't be throttled all the way down to zero,
        self.max_thrust = 400000.0  # N
        self.min_thrust = 150000.0  # N

        # ------ Gravity acting on the rocket's MOTION -------------------------
        # This is the gravity vector's magnitude used in
        # dv/dt = thrust_term + gravity. For an Earth landing, it's the same
        # number as g0 above, but conceptually it's a different thing (enviroment
        # vs engine formula), so we keep it as its own attribute.
        self.g = 9.80665  # m/s^2


        # ------ Thrust-vector steering limit ----------------------------------
        # Maximum angle, in RADIANS, that the thrust direction is allowed to
        # tilt away from straight up, math.radians() converts degrees (which
        # humans think in) to radians (which the math/trig function need)
        self.max_tilt_angle = math.radians(20.0) # about 0.349 rad

        if config_path is not None:
            self._load_from_yaml(config_path)

    def _load_from_yaml(self, config_path):
        """
        Overides whichever of the defaults aboce are PRESENT in the
        YAML file at config_path - anything the file doesn't mention
        just keeps its hardcoded default from __init__() above.

        max_titl_angle_deg is given in DEGREES in the YAML (human-
        readable, same convention as the math.radians() call above) -
        coverted to radians here, same as __init__() does.
        """
        with open(config_path) as f:
            data = yaml.safe_load(f)

        if "initial_mass" in data:
            self.initial_mass = data["initial_mass"]
        if "dry_mass" in data:
            self.dry_mass = data["dry_mass"]
        if "isp" in data:
            self.isp = data["isp"]
        if "g0" in data:
            self.g0 = data["g0"]
        if "max_thrust" in data:
            self.max_thrust = data["max_thrust"]
        if "min_thrust" in data:
            self.min_thrust = data["min_thrust"]
        if "g" in data:
            self.g = data["g"]
        if "max_tilt_angle_deg" in data:
            self.max_tilt_angle = math.radians(data["max_tilt_angle_deg"])


