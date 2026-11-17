"""Sampled speed reference and continuous ideal motor equations, in SI."""
import numpy as np

from . import params as P


def speed_reference(command, previous, dt=P.drive_sample_time):
    thrust = np.clip(np.asarray(command).reshape(4), 0, 1) * P.max_thrust
    target = np.sqrt(thrust / P.kT_prop)
    previous = np.asarray(previous).reshape(4)
    change = P.speed_slew_limit * dt
    return previous + np.clip(target - previous, -change, change)


def motor_outputs(reference, speed):
    speed = np.asarray(speed).reshape(4)
    acceleration = (np.asarray(reference).reshape(4) - speed) / P.motor_time_constant
    torque = P.kQ_prop * speed * np.abs(speed) + P.J_total * acceleration
    return acceleration, torque
