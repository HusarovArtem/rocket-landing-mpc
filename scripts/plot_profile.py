import numpy as np
import matplotlib.pyplot as plt

from rocket_landing.controllers.pid import PIDController
from rocket_landing.params import VehicleParams
from rocket_landing.actuators import clamp_control
from rocket_landing.simulation import simulate, simulate_vertical_pid


def plot_fixed_thrust_profile():
    """
    Runs ONE simulation with a CONSTANT control (same thrust the whole
    time, straight up) and plots now height (z), vertical speed (vz) and
    mass (m) evolve over time, i three stacked subplots sharing the same
    time axis.
    """
    p = VehicleParams()
    control = clamp_control(np.array([200000.0, 0.0, 0.0]), p)

    # start at rest, 500m up, high enough to give a clearly visible
    # curve, low enough that the simulation finished in a handful
    # of seconds instead of minutes
    state0 = np.array([0.0, 0.0, 500.0, 0.0, 0.0, 0.0, p.initial_mass])

    times, states = simulate(state0, control, p, dt=0.05) # 50 ms


    height = states[:, 2] # select ALL z column values
    vertical_speed = states[:, 5]
    mass = states[:, 6]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(7, 8))

    axes[0].plot(times, height)
    axes[0].set_ylabel("height z [m]")
    axes[0].grid(True)

    axes[1].plot(times, vertical_speed)
    axes[1].set_ylabel("vertical speed vz [m/s]")
    axes[1].grid(True)

    axes[2].plot(times, mass)
    axes[2].set_ylabel("mass m [kg]")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(True)

    fig.suptitle("Fixed thrust = 200000 N, straight up, starting from rest at 500 m")
    plt.tight_layout()
    plt.show()


def plot_3d_trajectory():
    """
    Same idea as step 21 (one constant control, no controller), but now
    alpha and beta are NOT zero - the thrust is tilted on purpose, so the
    rocket drifts horizontally (in x AND y) WHILE it falls. We plot the
    resulting path as a signle 3D curve (x, y, z) instead of three separate
    time slots.

    Why this matters for the project: so far everything has moved in a straight
    vertical line (x=y=0 always). This is the first time we actually use the
    "sideways" part of our model (alpha, beta) and see it do smth. Landing a rocket
    that's drifting sideways is exactly the kind of problem the later PID/MPC controller
    has to solve - this plot is just us looking whjat "doing nothing about it" looks like.
    """
    p = VehicleParams()

    # alpha tilts thrust in the x-direction, beta tilts it in the y-direction
    control = clamp_control(np.array([200000.0, 0.15, 0.1]), p)

    # making a bit higher than plot_fixed_thrust_profile()
    # to give more time for drift to become visible
    state0 = np.array([0.0, 0.0, 800.0, 0.0, 0.0, 0.0, p.initial_mass])

    times, states = simulate(state0, control, p, dt=0.05)

    # selecting all values per its column for x,y,z
    x = states[:, 0]
    y = states[:, 1]
    z = states[:, 2]

    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111, projection="3d")


    ax.plot(x, y, z, label="trajectory")
    ax.scatter(x[0], y[0], z[0], color="green", s=60, label="start")
    ax.scatter(x[-1], y[-1], z[-1], color="red", s=60, label="landing") # -1 means last el from array

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z (height) [m]")
    ax.set_title("3D trajectory, tilted thrust (alpha=0.15, beta=0.1 rad)")
    ax.legend()

    plt.tight_layout()
    plt.show()


def plot_vertical_pid_run(kp, ki, kd, initial_vz =-50):
    """
    Debugging helper for the manual-tuning step: instead of only looking
    at the FINAL numbers (which hide WHEN and HOW the overshoot
    happender), this runs one closed-loop PID landing attempt and plots
    height and vertical speed across the WHOLE flight. Seeing the actual
    shape of the vz curve - does it overshoot once and come back? does it
    oscillate repeatedly? does it fly most of the wat pegged at max thrust?
    - is much more useful for tuning that guessing from a single end-of-flight
    snapshot.
    """
    p = VehicleParams()
    pid = PIDController(kp=kp, ki=ki, kd=kd)
    state0 = np.array([0.0, 0.0, 500.0, 0.0, 0.0, initial_vz, p.initial_mass])

    times, states = simulate_vertical_pid(state0, pid, p, dt=0.05)

    height = states[:, 2]
    vz = states[:, 5]
    mass = states[:, 6]


    fig, axes = plt. subplots(3, 1, sharex=True, figsize=(7,8))

    axes[0].plot(times, height)
    axes[0].axhline(0.0, color="gray", linestyle="--", linewidth=1)
    axes[0].set_ylabel("height z [m]")
    axes[0].grid(True)


    axes[1].plot(times, vz)
    axes[1].axhline(0.0, color="gray", linestyle="--", linewidth=1)
    axes[1].set_ylabel("vertical speed vz [m/s]")
    axes[1].grid(True)


    axes[2].plot(times, mass)
    axes[2].axhline(p.dry_mass, color="red", linestyle="--", linewidth=1, label="dry_mass")
    axes[2].set_ylabel("mass m [kg]")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(True)

    fig.suptitle(f"PID vertical landing attempt: kp={kp}, ki={ki}, kd={kd}")
    plt.tight_layout()
    plt.show()

    print("landed after:", times[-1], "s")
    print("touchdown height:", states[-1][2])
    print("touchdown vz:", states[-1][5])
    print("touchdown mass:", states[-1][6])

if __name__ == "__main__":
    plot_vertical_pid_run(kp=1500.0, ki=50.0, kd=800.0)