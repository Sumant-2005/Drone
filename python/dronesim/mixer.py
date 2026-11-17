"""Quad-X commands in FR, BL, FL, BR motor order."""
import numpy as np

from . import params as P


def mix(collective, axis_demand):
    differential = P.mixer_matrix @ np.asarray(axis_demand).reshape(3)
    return np.clip(collective + differential, 0, 1)
