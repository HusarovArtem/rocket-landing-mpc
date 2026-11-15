from idlelib.debugobj_r import remote_object_tree_item

import numpy as np

def free_fall_dynamics(state, params):
    """
    Returns the time-derivative of the state, assuming NO thrust at all -
    i.e. the rocket is simply falling under gravity, like a dropped rock.

    This is the SIMPLEST possible version of the equations of motion.
    We only add gravity here, to make sure we get that part right before
    mixing in thrust and fuel consumption(that happens in the step 12, when
    we write the real dynamics() function).
    """
    # Recall the state layout we decided in docs/assumption.md:
    #   state = [x, y, z, vx, vy, vz, m]
    #            0  1  2  3   4   5   6
    # state[3:6] slices out elements at index 3, 4, 5 - i.e. vx, vy, vz.
    # That's a 3-element nupmy array: the current velocity vector
    velocity = state[3:6]

    # Acceleration due to gravity only. Gravity always points straight DOWN,
    # along the negative z-axis (see docs/assumptions.md for why z is "up").
    # params.g is a positive number (9.80665), so we negate it here.
    acceleration = np.array([0.0, 0.0, 0.0 - params.g])

    # Mass doesn't change during free fall - there's no engine burning any fuel
    # so the rate of change of mass is exactly zero.
    mass_rate = 0.0

    # The function must return something with the SAME SHAPE as the state:
    #       d(position)/dt = velocity     (3 numbers)
    #       d(velocity)/dt = acceleration (3 numbers)
    #       d(mass)/dt     = mass_rate    (1 number)
    # np.concatenate glued these pieces into a single 7 element array,
    # in the same order as the state itself.
    return np.concatenate([velocity, acceleration, [mass_rate]])

def thrust_direction(alpha, beta):
    """
    Returns the UNIT vector (length 1) pointing in the direction the thrust
    is aimed, given the steering angles:

        alpha - tilt angle AROUND the y-axis (tilts the thrust east/west,
            i.e. changes its x-component)
        beta - tilt angle AROUND the  x-axis (titkts the thrust noth/south,
            i.e. changes its y-component)

    Both angles are measured in RADIANS, away from straight up (+z).
    When alpha = 0 and beta = 0, the result is exactly [0, 0, 1] - thrust
    pointing straight up, as expected for a rocket sitting still on its engine.

    See docs/assumptions.md for the control vector definition - this
    function computes the `d(alpha, beta)` term from that file.
    """
    # This specific formula (rather that some other way of combining two
    # angles into a 3D direction) is chosen bcs it is guaranteed, by the
    # trig identity sin^2 + cos^2 = 1, to always produce a vector of
    # length EXACTLY 1, no matter what alpha and beta are
    x = np.sin(alpha) * np.cos(beta)
    y = np.sin(beta)
    z = np.cos(alpha) * np.cos(beta)

    return np.array([x, y, z])

def dynamics(state, control, params, wind=None):
    """
    The REAL equations of motion: gravity + engine thrust + fuel burn.
    No air resistance yet (see docs/assumptions.md - that's delibaretely
    left out of this first version

    state = [x, y, z, vx, vy, vz, m] (see free_fall_dynamics for details)
    control = [thrust, alpha, beta]
        thrust - engine force magnitude, in NEWTONS (NOT yet clamped to
        [min_thrust, max_thrust] here - that constraint is enforced
        elsewhere, e.g. by whoever picks 'control', or later by the
        optimizer/controller. This function just computes physics
        for whatever thrust value it's given.)
    alpha, beta - steering angles, in RADIANS (see thrust_direction)
    """
    velocity = state[3:6]
    mass = state[6]

    thrust, alpha, beta = control

    # Gravity pulls straight sown, same as in the free_fall_dynamics - this
    # does NOT depend on mass (that's the classic "all objects fall at the
    # same rate" fact: gravity is an ACCELERATION, not a force here).
    gravity_acceleration = np.array([0.0, 0.0, 0.0 - params.g])



    # Thrust is a FORCE (in Newtons): magnitude `thrust` times the unit
    # direction vector from thrust_direction(). To turn a force into an
    # acceleration we divide by the CURRENT mass (Newton's second law,
    # a = F/m) this is why a rocket accelerates harder and harder as it
    # burns fuel and gets lighter, even with constant thrust.
    direction = thrust_direction(alpha, beta)
    thrust_acceleration = thrust * direction / mass

    # Total acceleration felt by the rocket's center of mass: both effects
    # just add up as vectors (gravity doesn't care about thrust, and thrust
    # doesn't care about gravity).
    acceleration = gravity_acceleration + thrust_acceleration

    if wind is not None:
        acceleration = acceleration + wind.acceleration()

    # Mass flow equation (see params.py comment on isp): the bigger the
    # thrust, the faster fuel is burned. mass_rate is NEGATIVE bcs mass
    # is decreasing (we are losing propellant, not gaining it).
    mass_rate = -thrust / (params.isp * params.g0)

    return np.concatenate([velocity, acceleration, [mass_rate]])